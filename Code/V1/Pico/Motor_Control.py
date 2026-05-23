import time
from machine import Pin, PWM

# --- Configuration ---
EXTERNAL_GEAR_REDUCTION = 1
MIN_STALL_POWER = 0.15

class JointController:
    def __init__(self, rpwm_pin, lpwm_pin, en_pin, enc_a_pin, enc_b_pin, gear_ratio, ppr, reverse, initial_angle):
        """
        gear_ratio: 99.5 (from specs)
        ppr: 28 (from specs)
        """
        # --- Hardware Setup ---
        self.en = Pin(en_pin, Pin.OUT)
        self.reverse = reverse

        # 20kHz is correct. It keeps the motors silent (above human hearing) 
        # and provides smoother torque than lower frequencies.
        self.forward_pwm = PWM(Pin(rpwm_pin))
        self.backward_pwm = PWM(Pin(lpwm_pin))
        self.forward_pwm.freq(1000)   # Dropped to 1000Hz for optocoupler stability
        self.backward_pwm.freq(1000)  # Dropped to 1000Hz for optocoupler stability
        
        self.en.value(1) # Turn on the H-Bridge
        
        # --- Dynamic Math using your Specs ---
        # gear_ratio (99.5) * ppr (28) = 2786.2 pulses per output revolution
        ticks_per_joint_rev = ppr * gear_ratio * EXTERNAL_GEAR_REDUCTION
        self.ticks_per_degree = ticks_per_joint_rev / 360.0

        # --- Encoder Setup ---
        self.enc_a = Pin(enc_a_pin, Pin.IN)
        self.enc_b = Pin(enc_b_pin, Pin.IN)
        self._steps = int(initial_angle * self.ticks_per_degree)
        self.enc_a.irq(trigger=Pin.IRQ_RISING, handler=self._encoder_isr)


        # --- PID Tuning for goBILDA ---
        # These motors have huge torque (133 kg.cm). 
        # You might need to LOWER Kp because they are so strong they can oscillate.
        self.kp = 0.8  # Start slightly lower than before
        self.ki = 0.02
        self.kd = 0.05 # Increased slightly to dampen that high torque
        
        self.prev_error = None
        self.integral = 0
        self.last_time = time.ticks_ms()

    def _encoder_isr(self, pin):
        # Quadrature logic: Check B pin state to determine direction
        if self.enc_b.value():
            self._steps += 1
        else:
            self._steps -= 1

    @property
    def current_angle(self):
        """Returns the actual angle of the leg joint in degrees."""
        return self._steps / self.ticks_per_degree

    def move_to(self, target_angle):
        """
        Calculates PID and drives the motor. 
        """
        now = time.ticks_ms()
        # dt in seconds
        dt = (time.ticks_diff(now, self.last_time)) / 1000.0
        if dt <= 0: return

        actual_target = -target_angle if self.reverse else target_angle

        # 1. Calculate Error
        current = self.current_angle
        error = actual_target - current
        
        # 2. PID Terms
        self.integral = max(-100, min(100, self.integral + (error * dt))) # Anti-windup
        
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
            output = MIN_STALL_POWER if output > 0 else -MIN_STALL_POWER

        # 5. Drive H-Bridge (MicroPython uses 0-65535 for duty cycle)
        pwr = max(-1.0, min(1.0, output))
        duty = int(abs(pwr) * 65535)
        
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