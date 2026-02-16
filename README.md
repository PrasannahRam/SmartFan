🌬️ IoT-Based Smart Fan
📌 Overview

An intelligent IoT-based Smart Fan system designed to automatically adjust fan speed based on real-time temperature and human presence detection. The system integrates sensor data, computer vision, and PWM motor control to optimize energy efficiency and user comfort.

🚀 Features

    🌡️ Automatic speed control using real-time temperature readings

    👤 Human detection using camera + YOLO machine learning model
    
    🔁 Motion detection using HC-SR501 PIR sensor
    
    ⚡ PWM-based motor speed control
    
    🧠 Intelligent decision-making logic implemented in Python
    
    📡 Designed for energy-efficient IoT-based automation

🛠️ Technologies Used
  Hardware

    Raspberry Pi
    
    Temperature Sensor
    
    HC-SR501 PIR Motion Sensor
    
    Camera Module

    DC Motor / Fan with PWM Control

  Software

    Python
    
    OpenCV
    
    YOLO (You Only Look Once) Model
    
    GPIO Control (Raspberry Pi)

🧠 System Architecture

    Temperature sensor continuously monitors ambient temperature.
    
    PIR sensor and camera detect human presence.
    
    YOLO model processes camera frames to confirm human detection.
    
    Control algorithm determines optimal fan speed.
    
    PWM signal adjusts motor speed accordingly.

🎯 Objectives

    Reduce unnecessary power consumption
    
    Improve automation using AI-based human detection
    
    Integrate IoT, embedded systems, and machine learning into a practical application

📈 Future Improvements

    Mobile app integration for remote monitoring
    
    Cloud-based data logging
    
    Adaptive learning for personalized comfort control
