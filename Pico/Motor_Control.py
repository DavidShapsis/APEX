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

        self.forward_pwm = PWM(Pin(rpwm_pin))
        self.backward_pwm = PWM(Pin(lpwm_pin))
        self.forward_pwm.freq(1000)   # Dropped to 1000Hz for optocoupler stability
        self.backward_pwm.freq(1000)  # Dropped to 1000Hz for optocoupler stability
        
        self.en.value(1) # Turn on the H-Bridge
        
        # --- Dynamic Math using your Specs ---
        ticks_per_joint_rev = ppr * gear_ratio * EXTERNAL_GEAR_REDUCTION
        self.ticks_per_degree = ticks_per_joint_rev / 360.0

        # --- Encoder Setup ---
        self.enc_a = Pin(enc_a_pin, Pin.IN)
        self.enc_b = Pin(enc_b_pin, Pin.IN)
        
        # FIX 1: If hardware is reversed, our initial starting position pulses must be inverted
        raw_initial_steps = int(initial_angle * self.ticks_per_degree)
        self._steps = -raw_initial_steps if self.reverse else raw_initial_steps
        
        self.enc_a.irq(trigger=Pin.IRQ_RISING, handler=self._encoder_isr)

        # --- PID Tuning for goBILDA ---
        self.kp = 0.8  
        self.ki = 0.02
        self.kd = 0.05 
        
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
        now = time.ticks_ms()
        dt = (time.ticks_diff(now, self.last_time)) / 1000.0
        if dt <= 0: return

        # 1. Calculate Error (using the standardized target and standardized current_angle)
        current = self.current_angle
        error = target_angle - current
        
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