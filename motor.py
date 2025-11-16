import os

import RPi.GPIO as GPIO
import time
import cv2
from ultralytics import YOLO

model = YOLO('yolov8n.pt')  # Pretrained YOLOv8 Nano

# Define L298N pins
IN1 = 17  # GPIO pin connected to IN1
IN2 = 27  # GPIO pin connected to IN2
ENA = 22  # Optional: ENA pin, tie to 5V or use GPIO if you want control

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
# GPIO.setup(ENA, GPIO.OUT)  # Uncomment if using ENA pin via GPIO

# Optional: enable motor if using GPIO for ENA
# GPIO.output(ENA, GPIO.HIGH)

def human_detected():
    print("Detecting humans...")
    camera = cv2.VideoCapture("http://192.168.142.83:8080/video")

    ret, frame = camera.read()
    if not ret:
        print("Camera error")
        camera.release()
        return {"left": 0, "right": 0, "center": 0, "total": 0}

    results = model(frame, verbose=False)

    height, width, _ = frame.shape
    screen_range = width // 3

    left = right = center = 0

    for box in results[0].boxes:
        cls = int(box.cls[0])
        label = model.names[cls]

        if label.lower() == "person":
            # Get bounding box center x coordinate
            x1, y1, x2, y2 = box.xyxy[0]
            person_center_x = (x1 + x2) / 2

            # Determine side
            if person_center_x < screen_range:
                left += 1
            elif person_center_x < screen_range * 2:
                center += 1
            else:
                right += 1

    camera.release()

    total = left + center + right
    print(f"Detected {total} human(s): Left={left}, Center={center}, Right={right}")

    return {"left": left, "right": right, "center": center, "total": total}


def turn_left():
    print("turning left")
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)

def turn_right():
    print("turning right")
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)

def motor_stop():
    print("Motor Stop")
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    time.sleep(1)

try:
    while True:
        person_here = human_detected()

        # left  = human_detected()["left"]
        # right = human_detected()["right"]
        # center = human_detected()["center"]
        total = person_here['total']

        max_side = max(["left", "right", "center"], key=lambda k: person_here[k])

        if total == 0:
            motor_stop()
        elif max_side == 'left':
            turn_left()
        elif max_side == 'right':
            turn_right()
        elif max_side == 'center':
            motor_stop()



        os.system('cls' if os.name == 'nt' else 'clear')

        # Motor stop


except KeyboardInterrupt:
    print("Stopping motor and cleaning up GPIO")
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.cleanup()

