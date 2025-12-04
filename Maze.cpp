from machine import Pin, PWM, I2C
import utime
import vl53l0x

# ====== Motor Pins ======
IN1, IN2 = 10, 11
IN3, IN4 = 8, 7

# ====== Encoder Pins ======
encLA = Pin(3, Pin.IN, Pin.PULL_UP)
encLB = Pin(4, Pin.IN, Pin.PULL_UP)
encRA = Pin(12, Pin.IN, Pin.PULL_UP)
encRB = Pin(13, Pin.IN, Pin.PULL_UP)

SIDE_OPEN_THRESHOLD = 200
FRONT_BLOCKED_THRESHOLD = 150
SIDE_ALIGNMENT_MIN = 65
SIDE_ALIGNMENT_MAX = 135
SIDE_ALIGNMENT_TARGET = 100  # Ideal center distance

# ====== Sensor Setup ======
XSHUT_PINS = [21, 20, 19, 18, 17]
SENSOR_NAMES = ['left', 'left_front', 'front', 'right_front', 'right']
ADDRESSES = [0x30, 0x31, 0x32, 0x33, 0x34]

# ====== Thresholds ======
SIDE_OPEN_THRESHOLD = 200
FRONT_BLOCKED_THRESHOLD = 130

# ====== Speed / Control ======
BASE_SPEED = 25000
MIN_SPEED = 10000
MAX_SPEED = 65000
TURN_SPEED = 30000
TURN_TICKS_90 = 140

pid_integral = 0
pid_last_error = 0

KP = 10
KI = 0
KD = 300

# ====== Junction Instructions ======
class JunctionAction:
    TURN_LEFT = 0
    TURN_RIGHT = 1
    MOVE_FORWARD = 2

junctions = [
    JunctionAction.TURN_RIGHT,
    JunctionAction.TURN_RIGHT,
    JunctionAction.TURN_LEFT,
    JunctionAction.TURN_LEFT,
    JunctionAction.TURN_LEFT,
    JunctionAction.TURN_RIGHT,
    JunctionAction.TURN_RIGHT,
    JunctionAction.TURN_LEFT,
    JunctionAction.MOVE_FORWARD,
    JunctionAction.TURN_LEFT,
]

# ====== Global State ======
current_junction = 0
sensors = {}
left_ticks = 0
right_ticks = 0

# ====== Motor PWM Setup ======
pwm_L1 = PWM(Pin(IN1)); pwm_L1.freq(1000)
pwm_L2 = PWM(Pin(IN2)); pwm_L2.freq(1000)
pwm_R1 = PWM(Pin(IN3)); pwm_R1.freq(1000)
pwm_R2 = PWM(Pin(IN4)); pwm_R2.freq(1000)

# ====== Utility Functions ======
def _clamp(val, lo, hi):
    return max(lo, min(val, hi))

def motor_left(speed):
    speed = int(_clamp(speed, -MAX_SPEED, MAX_SPEED))
    pwm_L1.duty_u16(speed if speed > 0 else 0)
    pwm_L2.duty_u16(-speed if speed < 0 else 0)

def motor_right(speed):
    speed = int(_clamp(speed, -MAX_SPEED, MAX_SPEED))
    pwm_R1.duty_u16(speed if speed > 0 else 0)
    pwm_R2.duty_u16(-speed if speed < 0 else 0)

def stop_motors():
    motor_left(0)
    motor_right(0)

# ====== Encoder Callbacks ======
def left_encoder(pin):
    global left_ticks
    left_ticks += 1 if encLB.value() else -1

def right_encoder(pin):
    global right_ticks
    right_ticks += 1 if not encRB.value() else -1

encLA.irq(trigger=Pin.IRQ_RISING, handler=left_encoder)
encRA.irq(trigger=Pin.IRQ_RISING, handler=right_encoder)

# ====== Turning Using Encoders ======
def turn_by_ticks(ticks, speed=TURN_SPEED, left_ccw=True):
    global left_ticks, right_ticks
    left_ticks = 0
    right_ticks = 0

    if left_ccw:
        motor_left(-speed)
        motor_right(speed)
    else:
        motor_left(speed)
        motor_right(-speed)

    while (abs(left_ticks) + abs(right_ticks)) / 2 < ticks:
        utime.sleep_ms(1)

    stop_motors()
    utime.sleep_ms(100)
    motor_left(BASE_SPEED)
    motor_right(BASE_SPEED)
    utime.sleep_ms(300)

# ====== Sensor Initialization ======
def init_sensors():
    global sensors
    i2c = I2C(1, scl=Pin(15), sda=Pin(14), freq=400000)
    xshuts = [Pin(pin, Pin.OUT) for pin in XSHUT_PINS]

    for p in xshuts:
        p.value(0)
    utime.sleep_ms(50)

    for i, pin in enumerate(xshuts):
        pin.value(1)
        utime.sleep_ms(100)
        sensor = vl53l0x.VL53L0X(i2c)
        sensor.set_address(ADDRESSES[i])
        sensors[SENSOR_NAMES[i]] = sensor

# ====== Detect Junctions (With Sensor Offsets) ======
def detect_junction():
    dL = sensors['left'].read() 
    dR = sensors['right'].read() 
    dF = sensors['front'].read()  # No offset

    return dL > SIDE_OPEN_THRESHOLD or dR > SIDE_OPEN_THRESHOLD or dF < FRONT_BLOCKED_THRESHOLD

# ====== Main Loop ======
def main_loop():
    global current_junction

    init_sensors()
    print("Sensors initialized. Starting main loop.")

    while current_junction < len(junctions):
        if detect_junction():
            print("Junction Detected!")
            stop_motors()
            utime.sleep_ms(100)
            motor_left(-BASE_SPEED)
            motor_right(-BASE_SPEED)
            utime.sleep_ms(120)

            action = junctions[current_junction]
            print(f"Junction {current_junction + 1} action: {action}")

            if action == JunctionAction.TURN_LEFT:
                motor_left(BASE_SPEED)
                motor_right(BASE_SPEED)
                utime.sleep_ms(20)
                turn_by_ticks(TURN_TICKS_90, left_ccw=True)
                motor_left(BASE_SPEED)
                motor_right(BASE_SPEED)
                utime.sleep_ms(100)
            elif action == JunctionAction.TURN_RIGHT:
                motor_left(BASE_SPEED)
                motor_right(BASE_SPEED)
                utime.sleep_ms(20)
                turn_by_ticks(TURN_TICKS_90, left_ccw=False)
                motor_left(BASE_SPEED)
                motor_right(BASE_SPEED)
                utime.sleep_ms(100)
            elif action == JunctionAction.MOVE_FORWARD:
                motor_left(BASE_SPEED)
                motor_right(BASE_SPEED)
                utime.sleep_ms(300)
                stop_motors()

            current_junction += 1
            motor_left(BASE_SPEED)
            motor_right(BASE_SPEED)
            utime.sleep_ms(200)

        else:
            # Keep moving forward while no junction
            motor_left(27000)
            motor_right(25000)
            
            

        utime.sleep(0.01)

    print("All junctions handled. Stopping.")
    
def forward_PID():
    global pid_integral, pid_last_error

    dL = sensors['left'].read()
    dR = sensors['right'].read()

    # Stop if out of safe bounds
    if not (SIDE_ALIGNMENT_MIN <= dL <= SIDE_ALIGNMENT_MAX and SIDE_ALIGNMENT_MIN <= dR <= SIDE_ALIGNMENT_MAX):
        print(f"Side out of bounds (L: {dL}, R: {dR}) — stopping")
        stop_motors()
        return

    error = dL - dR
    pid_integral += error
    derivative = error - pid_last_error
    pid_last_error = error

    correction = KP * error + KI * pid_integral + KD * derivative
    correction = int(correction)

    left_speed = _clamp(BASE_SPEED - correction, MIN_SPEED, MAX_SPEED)
    right_speed = _clamp(BASE_SPEED + correction, MIN_SPEED, MAX_SPEED)

    motor_left(left_speed)
    motor_right(right_speed)

# ====== Run Program ======
try:
    main_loop()
except KeyboardInterrupt:
    stop_motors()
    print("Stopped by user.")
