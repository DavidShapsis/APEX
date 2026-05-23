import time
from machine import Pin, PWM

# Initialize Enable pins
r_en = Pin(18, Pin.OUT)
l_en = Pin(19, Pin.OUT)

# Initialize PWM pins for speed control
rpwm = PWM(Pin(16))
lpwm = PWM(Pin(17))

# Set PWM frequency (1000Hz is standard for BTS7960)
rpwm.freq(1000)
lpwm.freq(1000)

def set_motor_speed(direction, speed):
    """
    Control motor direction and speed.
    direction: 'forward' or 'reverse'
    speed: 0 (stopped) to 65535 (max speed)
    """
    # Ensure both EN pins are enabled
    r_en.value(1)
    l_en.value(1)
    
    if direction == 'forward':
        lpwm.duty_u16(0)       # Stop reverse
        rpwm.duty_u16(speed)   # Set forward speed
    elif direction == 'reverse':
        rpwm.duty_u16(0)       # Stop forward
        lpwm.duty_u16(speed)   # Set reverse speed
    else:
        # Brake
        rpwm.duty_u16(0)
        lpwm.duty_u16(0)

# Main Test Loop
try:
    print("Motor Test Started")
    
    # 1. Ramp up forward speed
    print("Forward...")
    for duty in range(0, 65535, 500):
        set_motor_speed('forward', duty)
        time.sleep(0.05)
    time.sleep(2)
    
    # 2. Stop
    set_motor_speed('stop', 0)
    time.sleep(1)
    
    # 3. Ramp up reverse speed
    print("Reverse...")
    for duty in range(0, 65535, 500):
        set_motor_speed('reverse', duty)
        time.sleep(0.05)
    time.sleep(2)

finally:
    # Safely stop motors and disable driver when script exits
    set_motor_speed('stop', 0)
    r_en.value(0)
    l_en.value(0)
    print("Motor Test Finished")