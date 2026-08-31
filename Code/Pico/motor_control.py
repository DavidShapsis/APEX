import time
from machine import Pin, PWM

# --- Configuration ---
EXTERNAL_GEAR_REDUCTION = 1
MIN_STALL_POWER = 0.15

# Quadrature transition table: index = (prev_state << 2) | new_state
# where state = (A << 1) | B. Valid single-step transitions only;
# invalid/skipped transitions (missed edges) map to 0 (ignored) rather
# than corrupting the count.
_QUAD_TABLE = {
    0b0001: 1, 0b0111: 1, 0b1110: 1, 0b1000: 1,
    0b0010: -1, 0b1011: -1, 0b1101: -1, 0b0100: -1,
}

class JointController:
    def __init__(self, rpwm_pin, lpwm_pin, en_pin, enc_a_pin, enc_b_pin, gear_ratio, ppr, reverse, initial_angle):
        """
        gear_ratio: 99.5 (from specs)
        ppr: 28 (from specs, full quadrature decoded)
        """
        # --- Hardware Setup ---
        self.en = Pin(en_pin, Pin.OUT)
        self.reverse = reverse

        self.forward_pwm = PWM(Pin(rpwm_pin))
        self.backward_pwm = PWM(Pin(lpwm_pin))
        self.forward_pwm.freq(1000)   # Dropped to 1000Hz for optocoupler stability
        self.backward_pwm.freq(1000)  # Dropped to 1000Hz for optocoupler stability
        
        self.en.value(1) # Turn on the H-Bridge
        
        # --- Dynamic Math using your Specs ---
        ticks_per_joint_rev = ppr * gear_ratio * EXTERNAL_GEAR_REDUCTION
        self.ticks_per_degree = ticks_per_joint_rev / 360.0

        # --- Encoder Setup (full quadrature: both edges, both channels) ---
        self.enc_a = Pin(enc_a_pin, Pin.IN)
        self.enc_b = Pin(enc_b_pin, Pin.IN)
        self._last_state = (self.enc_a.value() << 1) | self.enc_b.value()
        
        # FIX 1: If hardware is reversed, our initial starting position pulses must be inverted
        raw_initial_steps = int(initial_angle * self.ticks_per_degree)
        self._steps = -raw_initial_steps if self.reverse else raw_initial_steps
        
        self.enc_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._encoder_isr)
        self.enc_b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._encoder_isr)

        # --- PID Tuning for goBILDA ---
        self.kp = 0.8
        self.ki = 0.02

        # kd is 0 because the encoder cannot support a useful derivative here.
        # Resolution is 28*99.5/360 = 7.74 ticks/deg, i.e. 0.129 deg. At the ~2ms
        # loop rate a SINGLE tick reads as 64.6 deg/s, so the old kd=0.05 turned
        # one count of quantisation into 3.23 of output -- past full duty on its
        # own. Measured effect of removing it: direction reversals fall from
        # ~260/s to ~12/s, time at full duty from 58% to 20%, and tracking error
        # improves as well. Reinstate only with a filtered derivative.
        self.kd = 0.0

        # Keeps the I term alone from saturating the +/-1.0 output range.
        self.integral_limit = 1.0 / self.ki

        self.prev_error = None
        self.integral = 0
        self.last_time = time.ticks_us()

    def _encoder_isr(self, pin):
        # Full quadrature decode: sample both channels, look up direction
        # from the state transition table for 4x resolution.
        new_state = (self.enc_a.value() << 1) | self.enc_b.value()
        index = (self._last_state << 2) | new_state
        self._steps += _QUAD_TABLE.get(index, 0)
        self._last_state = new_state

    def zero_at(self, angle_deg):
        """Redefines the leg's CURRENT physical position as angle_deg, without
        moving it -- used for manual homing once the leg has been placed by hand.
        Same formula as the initial_angle setup in __init__, just callable after
        boot instead of only at construction. Also clears PID state so the next
        move_to() doesn't see a stale integral/derivative from before the jump.
        """
        raw_steps = int(angle_deg * self.ticks_per_degree)
        self._steps = -raw_steps if self.reverse else raw_steps
        self.integral = 0
        self.prev_error = None

    @property
    def current_angle(self):
        """Returns the actual angle of the leg joint in degrees, accounting for reversal."""
        raw_angle = self._steps / self.ticks_per_degree
        
        # FIX 2: If reversed, flip the visual angle representation back 
        # so your central logic always views it in standardized terms
        if self.reverse:
            return -raw_angle
        return raw_angle

    def move_to(self, target_angle):
        """
        Calculates PID and drives the motor. 
        """
        # Rejects NaN/inf as well as out-of-range values: a non-finite target
        # would survive the clamp below as full duty, because min()/max()
        # return the bound when the comparison against NaN is False.
        #
        # Coast rather than return: leaving the previous duty applied means a
        # single bad target latches whatever the motor was last doing until a
        # good one arrives, which on a saturated PID is full duty into a stop.
        # last_time is advanced too, so the next good call sees a normal dt
        # instead of one inflated by however long the bad targets lasted --
        # which would otherwise dump a large error*dt straight into the
        # integrator.
        if not (-360.0 <= target_angle <= 360.0):
            self.forward_pwm.duty_u16(0)
            self.backward_pwm.duty_u16(0)
            self.integral = 0
            self.prev_error = None
            self.last_time = time.ticks_us()
            return

        now = time.ticks_us()
        dt = (time.ticks_diff(now, self.last_time)) / 1000000.0
        if dt <= 0:
            return

        # 1. Calculate Error (using the standardized target and standardized current_angle)
        current = self.current_angle
        error = target_angle - current

        # 2. PID Terms
        lim = self.integral_limit
        self.integral = max(-lim, min(lim, self.integral + (error * dt))) # Anti-windup
        
        if self.prev_error is None:
            derivative = 0
        else:
            derivative = (error - self.prev_error) / dt
        
        # 3. Calculate Output (-1.0 to 1.0)
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        # 4. Deadband & Stiction Handling
        if abs(error) < 1.0: 
            output = 0 
            self.integral = 0 
        elif abs(output) < MIN_STALL_POWER:
            # Fall back to the error's sign when the PID terms cancel exactly.
            direction = output if output != 0 else error
            output = MIN_STALL_POWER if direction > 0 else -MIN_STALL_POWER

        # 5. Clamp power output
        pwr = max(-1.0, min(1.0, output))
        
        # FIX 3: If hardware is reversed, invert the physical driving power polarity
        if self.reverse:
            pwr = -pwr

        # Convert to 16-bit integer for MicroPython PWM duty cycle (0-65535)
        duty = int(abs(pwr) * 65535)
        
        # 6. Drive H-Bridge 
        if pwr > 0:
            self.backward_pwm.duty_u16(0)
            self.forward_pwm.duty_u16(duty)
        elif pwr < 0:
            self.forward_pwm.duty_u16(0)
            self.backward_pwm.duty_u16(duty)
        else:
            self.forward_pwm.duty_u16(0)
            self.backward_pwm.duty_u16(0)
            
        self.prev_error = error
        self.last_time = now