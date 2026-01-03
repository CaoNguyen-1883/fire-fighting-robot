"""
Configuration file for Backend Server
Centralized configuration management
"""

# Flask Server Configuration
FLASK_HOST = "0.0.0.0"
FLASK_PORT = 5000
SECRET_KEY = "fire-robot-secret-2025"

# MQTT Broker Configuration
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_KEEPALIVE = 60
MQTT_CLIENT_ID = "FireRobotBackend"

# MQTT Topics
TOPIC_MOTOR_CONTROL = "robot/control/motor"
TOPIC_PUMP_CONTROL = "robot/control/pump"
TOPIC_STATUS = "robot/status"
TOPIC_SENSOR_DISTANCE = "robot/sensors/distance"
TOPIC_SENSOR_FLAME = "robot/sensors/flame"

# ESP32-CAM Configuration
ESP32_CAM_URL = "http://192.168.1.7"  # IP từ Serial Monitor

# AI Performance Configuration
# Adjust these based on your CPU performance:
# - Fast CPU (i7/i9): process_every_n_frames = 2 (~15 FPS AI)
# - Medium CPU (i5): process_every_n_frames = 3 (~10 FPS AI)  [DEFAULT]
# - Slow CPU (i3/Celeron): process_every_n_frames = 5 (~6 FPS AI)
AI_PROCESS_EVERY_N_FRAMES = 3  # Process 1 out of every N frames
AI_CONFIDENCE_THRESHOLD = 0.5  # YOLO detection confidence (0.0-1.0)

# WebSocket Configuration
CORS_ALLOWED_ORIGINS = "*"

# Logging Configuration
LOG_LEVEL = "INFO"
LOG_FORMAT = "%(asctime)s [%(levelname)s] %(name)s: %(message)s"
LOG_DATE_FORMAT = "%H:%M:%S"
