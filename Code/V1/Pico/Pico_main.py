import time
from machine import UART, Pin
from FSR import FSR
from Motor_Control import JointController
print("Imports successful")

# --- Setup Joints ---
# Using your class parameters - Updated for goBILDA 5203-2402-0100
roll_j  = JointController(rpwm_pin=3, lpwm_pin=2, en_pin=6, enc_a_pin=4, enc_b_pin=5, 
                          gear_ratio=99.5, ppr=28, 
                          reverse=False, 
                          initial_angle=0)
pitch_j = JointController(rpwm_pin=11, lpwm_pin=10, en_pin=7, enc_a_pin=8, enc_b_pin=9, 
                          gear_ratio=99.5, ppr=28, 
                          reverse=False, 
                          initial_angle=0)
knee_j  = JointController(rpwm_pin=15, lpwm_pin=14, en_pin=22, enc_a_pin=12, enc_b_pin=13, 
                          gear_ratio=99.5, ppr=28, 
                          reverse=False, 
                          initial_angle=0)
print("Joint Controllers setup succesful")

fsrs = [FSR(16), FSR(17), FSR(18), FSR(19)]
print("FSR setup successful")

uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
print("UART setup successful")
print("--------------------")

# --- State Management ---
gait_buffer = []
is_receiving = False
has_aborted = False # NEW: Track if we are currently in an error state
current_step_index = 0
STEP_TICK_MS = 50
last_step_time = time.ticks_ms() 

# FIXED: Moved outside the while loop so targets don't reset every millisecond!
current_targets = [0.0, 0.0, 0.0]

while True:
    # 1. READ UART
    if uart.any():
        line = uart.readline()
        if line:
            line = line.decode('utf-8').strip()
        else:
            line = ""
        if line == "START":
            gait_buffer = []
            is_receiving = True
            has_aborted = False # Reset the abort flag when a new gait starts
        elif line == "END":
            is_receiving = False
            current_step_index = 0
            last_step_time = time.ticks_ms()
        elif is_receiving:
            try:
                parts = [float(x) for x in line.split(',')]
                if len(parts) == 3: gait_buffer.append(parts)
            except: pass

    # 2. GROUND CHECK (The Abort Logic)
    any_touchdown = any(f.state for f in fsrs)
    
    # NEW: Only abort if we are actually moving (buffer exists) 
    # and we aren't in the middle of a UART download.
    if any_touchdown and not has_aborted and gait_buffer and not is_receiving:
        msg = f"ABORTED,{roll_j.current_angle},{pitch_j.current_angle},{knee_j.current_angle}\n"
        uart.write(msg)
        has_aborted = True 
        gait_buffer = []
        # ADD THESE:
        roll_j.integral = 0
        pitch_j.integral = 0
        knee_j.integral = 0

    # 3. CHOOSE TARGETS (Every 50ms)
    if gait_buffer and not is_receiving and not has_aborted:
        if time.ticks_diff(time.ticks_ms(), last_step_time) > STEP_TICK_MS:
            current_step_index = (current_step_index + 1) % len(gait_buffer)
            last_step_time = time.ticks_ms()

            # Pull the current targets from the buffer
            current_targets = gait_buffer[current_step_index]

            print(f"Targets: {current_targets}")

    # 4. EXECUTE CLOSED LOOP PID UPDATES (Every Single Loop iteration!)
    if gait_buffer and not is_receiving and not has_aborted:
        roll_j.move_to(current_targets[0])
        pitch_j.move_to(current_targets[1])
        knee_j.move_to(current_targets[2])
    else:
        # Safety fallback: Force motor duties to 0 if aborted or downloading
        roll_j.forward_pwm.duty_u16(0)
        roll_j.backward_pwm.duty_u16(0)
        pitch_j.forward_pwm.duty_u16(0)
        pitch_j.backward_pwm.duty_u16(0)
        knee_j.forward_pwm.duty_u16(0)
        knee_j.backward_pwm.duty_u16(0)

    time.sleep_ms(1)