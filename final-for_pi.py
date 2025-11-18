import RPi.GPIO as GPIO
import time
import cv2
from ultralytics import YOLO
import adafruit_dht
import board

# -------------------------------
# GPIO setup
# -------------------------------
GPIO.setmode(GPIO.BCM)

# -------------------------------
# PWM Fan module setup (6-terminal)
# -------------------------------
FAN_PWM_PIN = 18  # Connect this to VTRIG/PWM on your PWM module
GPIO.setup(FAN_PWM_PIN, GPIO.OUT)
fan_pwm = GPIO.PWM(FAN_PWM_PIN, 100)  # 100 Hz PWM frequency
fan_pwm.start(0)  # Start with fan OFF

# -------------------------------
# Ultrasonic sensor (HC-SR04)
# -------------------------------
TRIG = 23
ECHO = 24
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# -------------------------------
# PIR motion sensor
# -------------------------------
PIR_PIN = 17
GPIO.setup(PIR_PIN, GPIO.IN)

# -------------------------------
# DHT22 temperature sensor
# -------------------------------
dht_sensor = adafruit_dht.DHT22(board.D4)

# -------------------------------
# YOLO model for human detection
# -------------------------------
model = YOLO('yolov8n.pt')

recheck_interval = 10
last_detected_time = 0
# -------------------------------
# Helper functions
# -------------------------------
def get_distance():
    """Measure distance from HC-SR04 (cm). Returns None if failed."""
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    start_time = time.time()
    timeout = start_time + 0.04  # 40ms timeout

    while GPIO.input(ECHO) == 0:
        start_time = time.time()
        if start_time > timeout:
            return None
    stop_time = time.time()
    while GPIO.input(ECHO) == 1:
        stop_time = time.time()
        if stop_time > timeout:
            return None

    distance = ((stop_time - start_time) * 34300) / 2
    return distance

def human_detected():
    """Detect number of people using YOLOv8. Returns 0 if camera fails."""

    camera = cv2.VideoCapture("http://192.168.142.83:8080/video")  # IP camera stream

    try:
        ret, frame = camera.read()
        if not ret:
            camera.release()
            return 0   # no frame → no people

        results = model(frame, verbose=False)

        people_count = 0

        for box in results[0].boxes:
            cls = int(box.cls[0])
            label = model.names[cls]

            if label.lower() == "person":
                people_count += 1

        camera.release()
        return people_count

    except Exception as e:
        print(f"YOLO error: {e}")
        camera.release()
        return 0

def read_temperature(retries=3):
    """Read DHT22 temperature with retries. Returns None if all fail."""
    for _ in range(retries):
        try:
            temp = dht_sensor.temperature
            if temp is not None:
                return temp
        except RuntimeError:
            time.sleep(0.2)
    return None

# -------------------------------
# Main loop
# -------------------------------

def calc_fan_duty(temp_c: float, num_people: int, avg_distance_cm: float):

    TEMP_CUTOFF = 27.0          # Below this temperature, fan stays off
    TEMP_MAX = 35.0             # Max temperature for full fan speed
    MIN_DUTY = 10               # Minimum fan speed when running
    MAX_DISTANCE_CM = 400       # If people are farther than this, fan stays off
    MAX_PEOPLE_CAN_COVER = 3    # Scaling is designed for maximum 3 people

    # --- Conditions where the fan should be OFF ---
    if temp_c <= TEMP_CUTOFF or num_people == 0 or avg_distance_cm > MAX_DISTANCE_CM:
        return 0

    # Temperature difference from the cutoff
    temp_range = TEMP_MAX - TEMP_CUTOFF
    current_temp_diff = temp_c - TEMP_CUTOFF

    # Base fan speed depending only on temperature
    base_duty = MIN_DUTY        # Start with minimum
    if current_temp_diff >= temp_range:
        base_duty = 100         # Full speed at very high temperature
    elif current_temp_diff > 0:
        base_duty = (current_temp_diff / temp_range) * 100

    # Make sure base duty is not less than MIN_DUTY
    base_duty = max(base_duty, MIN_DUTY)

    # Extra speed added per person depending on distance
    PERSON_SCALING_PER_HEAD = (100 - MIN_DUTY) / MAX_PEOPLE_CAN_COVER * (avg_distance_cm / MAX_DISTANCE_CM)

    # Add scaling for additional people (first person already counted)
    scaling_factor = max(0, num_people - 1) * PERSON_SCALING_PER_HEAD

    # Final fan speed
    final_duty = base_duty + scaling_factor

    # Limit to 100%
    final_duty = int(min(final_duty, 100))

    return final_duty

def run_detection():
    global last_detected_time,temperature
    last_detected_time = time.time()
    dist = get_distance()
    person_here = human_detected()
    # Fan control logic
    duty_cycle = 0
    if temperature and temperature > 27 and person_here and dist and dist < 300:
        # Scale fan speed between 28°C → 10% and 37°C → 100%
        duty_cycle = calc_fan_duty(temperature,person_here,dist)

    # Smooth fan changes by limiting sudden jumps
    current_dc = getattr(fan_pwm, "current_duty", 0)
    while abs(duty_cycle - current_dc) > 20:  # limit jump to 20%
        duty_cycle = current_dc + (20 if duty_cycle > current_dc else -20)
        fan_pwm.ChangeDutyCycle(duty_cycle)
        current_dc = duty_cycle  # store last duty
        time.sleep(0.4)


    print(
        f"Temp={temperature}C | Dist={dist}cm | Human={person_here} | Fan Duty={duty_cycle}%")



try:
    while True:

        current_time = time.time()

        # Read sensors
        temperature = read_temperature()

        motion_detected = GPIO.input(PIR_PIN)

        if (current_time - last_detected_time > recheck_interval) or motion_detected:
            print("Running Detection")
            run_detection()




        time.sleep(2)

except KeyboardInterrupt:
    print("Exiting...")

finally:
    fan_pwm.stop()
    GPIO.cleanup()
    cv2.destroyAllWindows()