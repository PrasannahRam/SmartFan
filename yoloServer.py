from flask import Flask, jsonify
from ultralytics import YOLO
import cv2
import time

app = Flask(__name__)

model = YOLO("yolov8n.pt")


@app.route("/detect")
def detect():
    camera = cv2.VideoCapture("http://192.168.142.83:8080/video")
    ret, frame = camera.read()
    if not ret:
        return jsonify({"people": 0, "error": "no_frame"})

    frame = cv2.resize(frame, (320, 240))
    results = model(frame, device='cpu', verbose=False)

    people = 0
    for box in results[0].boxes:
        cls = int(box.cls[0])
        if cls == 0:  # 'person'
            people += 1

    return jsonify({
        "people": int(people),
        "timestamp": time.time()
    })

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5005)
