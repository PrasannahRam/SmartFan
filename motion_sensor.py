import RPi.GPIO as GPIO
import time

# --- GPIO Setup ---
PIR_PIN = 4  # Change this to the GPIO pin you connected the OUT pin of HC-SR501 to
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIR_PIN, GPIO.IN)

print("PIR Motion Sensor Test (press Ctrl+C to exit)")
time.sleep(2)  # Allow sensor to stabilize

try:
    while True:
        if GPIO.input(PIR_PIN):
            print("Motion detected!")
        else:
            print("No motion")
        time.sleep(1)

except KeyboardInterrupt:
    print("Exiting program...")
    GPIO.cleanup()
