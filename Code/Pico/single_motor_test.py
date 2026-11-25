from machine import Pin, PWM

rpwm = PWM(Pin(3))
rpwm.duty_u16(0)

lpwm = PWM(Pin(2))
lpwm.duty_u16(0)

en = Pin(6, Pin.OUT)
en.value(0)

print("All motor outputs forced off")