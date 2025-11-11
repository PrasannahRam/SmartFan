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

RELAY_PIN = 27
TRIG = 17
ECHO = 18

GPIO.setup(RELAY_PIN, GPIO.OUT)
GPIO.output(RELAY_PIN, GPIO.LOW)  # Fan OFF initially

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# -------------------------------
# Sensor setup
# -------------------------------
dht_sensor = adafruit_dht.DHT22(board.D4)  # DHT22 on GPIO4

# -------------------------------
# YOLO setup
# -------------------------------
model = YOLO('yolov8n.pt')  # Pretrained YOLOv8 Nano
  # Default camera

# -------------------------------
# Helper functions
# -------------------------------

def get_distance():
    """Measure distance from HC-SR04."""
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    start_time = time.time()
    stop_time = time.time()

    # Wait for echo start
    while GPIO.input(ECHO) == 0:
        start_time = time.time()
    # Wait for echo end
    while GPIO.input(ECHO) == 1:
        stop_time = time.time()

    elapsed = stop_time - start_time
    distance = (elapsed * 34300) / 2  # in centimeters
    return distance

def human_detected():
    print("detecting human")
    camera = cv2.VideoCapture("http://192.168.142.83:8080/video")
    """Return True if YOLO detects a person in frame."""
    
    ret, frame = camera.read()
    results = model(frame, verbose=False)
    if not ret:
        print("Camera error")
        return False
    for box in results[0].boxes:
        cls = int(box.cls[0])
        label = model.names[cls]
        if label.lower() == "person":
            print("person detectod")
            return True
    camera.release()
    return False

# -------------------------------
# Main loop
# -------------------------------
try:
    while True:
        # 1. Read temperature
        try:
            print("getting temp")
            temperature = dht_sensor.temperature
            #print(f"tamp is:{temperature}")
        except RuntimeError:
            temperature = None

        # 2. Measure distance
        dist = get_distance()

        # 3. Capture camera frame
        

        # 4. Detect human
        person_here = human_detected()

        # 5. Logic conditions
        print(dist)
        if temperature and temperature > 27 and person_here and dist < 300:
            GPIO.output(RELAY_PIN, GPIO.HIGH)  # Turn fan ON
            print("Fan ON | Temp = {:.1f} C | Dist = {:.1f} cm | Human = {}".format(
                temperature, dist, person_here))
        else:
            GPIO.output(RELAY_PIN, GPIO.LOW)  # Turn fan OFF
            print("Fan OFF | Temp = {} | Dist = {:.1f} cm | Human = {}".format(
                temperature, dist, person_here))

        # Optional: show camera window (press q to quit)
        #cv2.imshow("Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(2)

except KeyboardInterrupt:
    print("Exiting...")

finally:
    cv2.destroyAllWindows()
    GPIO.cleanup()
