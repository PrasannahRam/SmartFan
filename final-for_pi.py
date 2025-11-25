import requests
import RPi.GPIO as GPIO
import time
import adafruit_dht
import board


print("Starting Smart Fan System...")

# -------------------------------
# GPIO setup
# -------------------------------
GPIO.setmode(GPIO.BCM)

# -------------------------------
# PWM Fan setup
# -------------------------------
FAN_PWM_PIN = 18
GPIO.setup(FAN_PWM_PIN, GPIO.OUT)
fan_pwm = GPIO.PWM(FAN_PWM_PIN, 100)  # 100 Hz
fan_pwm.start(0)
fan_last_duty = 0

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
dht_sensor = adafruit_dht.DHT22(board.D4, use_pulseio=False)

# -------------------------------
# Remote YOLO backend
# -------------------------------
recheck_interval = 10
last_detected_time = 0


# -------------------------------
# Helper functions
# -------------------------------
def get_distance():
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    start_time = time.time()
    timeout = start_time + 0.04

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


def get_people_count():
    try:
        r = requests.get("http://192.168.142.220:5005/detect", timeout=2)
        data = r.json()
        return data.get("people", 0)
    except:
        print("Error: YOLO backend unreachable.")
        return 0


def read_temperature(retries=3):
    for _ in range(retries):
        try:
            temp = dht_sensor.temperature
            if temp is not None:
                return temp
        except:
            print("Error: DHT22 read failed.")
            time.sleep(0.2)
    return None


def calc_fan_duty(temp_c: float, num_people: int, avg_distance_cm: float):
    TEMP_CUTOFF = 25.0
    TEMP_MAX = 35.0
    MIN_DUTY = 10
    MAX_DISTANCE_CM = 200
    MAX_PEOPLE_CAN_COVER = 3

    temp_range = TEMP_MAX - TEMP_CUTOFF
    current_temp_diff = temp_c - TEMP_CUTOFF

    duty_from_temp = MIN_DUTY
    if current_temp_diff > temp_range:
        duty_from_temp = 100
    elif current_temp_diff > 0:
        duty_from_temp = (current_temp_diff / temp_range) * 40

    PERSON_SCALING_PER_HEAD = (60 / MAX_PEOPLE_CAN_COVER) * (avg_distance_cm / MAX_DISTANCE_CM) ** 0.5
    duty_from_people = num_people * PERSON_SCALING_PER_HEAD

    final_duty = duty_from_temp + duty_from_people
    return int(min(final_duty, 100))


def run_detection():
    global last_detected_time, fan_last_duty
    last_detected_time = time.time()

    dist = get_distance()
    person_here = get_people_count()
    temperature = read_temperature()

    if dist is None:
        dist = 9999

    duty_cycle = 0
    if temperature and temperature > 27 and int(person_here) and dist < 300:
        duty_cycle = calc_fan_duty(temperature, person_here, dist)

    # Smooth changes
    while abs(duty_cycle - fan_last_duty) > 10:
        fan_last_duty += 10 if duty_cycle > fan_last_duty else -10
        fan_pwm.ChangeDutyCycle(fan_last_duty)
        time.sleep(0.2)

    if fan_last_duty < 10:
        fan_pwm.ChangeDutyCycle(duty_cycle)

    fan_last_duty = duty_cycle
    print(f"Temp={temperature}°C | Dist={dist:.1f}cm | People={person_here} | Fan={duty_cycle}%")

# -------------------------------
# Main loop
# -------------------------------
try:
    print("System Ready.")
    while True:
        current_time = time.time()
        motion_detected = GPIO.input(PIR_PIN)

        if motion_detected:
            run_detection()
        elif current_time - last_detected_time > recheck_interval:
            run_detection()

        time.sleep(1)

except KeyboardInterrupt:
    print("Shutting down...")
    pass

finally:
    fan_pwm.stop()
    GPIO.cleanup()
    print("GPIO Cleaned.")
