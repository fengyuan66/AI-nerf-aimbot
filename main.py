import cv2
import mediapipe as mp
import numpy as np
import serial
import serial.tools.list_ports
import time

def find_arduino_port():
    """Search for a port that likely belongs to the Arduino."""
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        # Look for common identifiers in the device name (adjust as needed)
        if "usbmodem" in port.device.lower() or "usbserial" in port.device.lower():
            return port.device
    return None

# === AUTO-DETECT ARDUINO PORT ===
arduino_port = find_arduino_port()
if not arduino_port:
    print("Could not find Arduino port! Please check your connection.")
    exit(1)
print("Found Arduino on port:", arduino_port)

# === SERIAL CONNECTION ===
try:
    ser = serial.Serial(arduino_port, 9600, timeout=2)
    time.sleep(2)  # Wait for connection to stabilize
except Exception as e:
    print("Failed to connect to Arduino:", e)
    exit(1)

# === Request default position from Arduino ===
ser.write(b"INIT\n")
default_line = ser.readline().decode().strip()
# Expected format: "DEFAULT,x,y"
if default_line.startswith("DEFAULT"):
    try:
        _, def_x, def_y = default_line.split(',')
        DEFAULT_X = int(def_x)
        DEFAULT_Y = int(def_y)
    except Exception as e:
        print("Error parsing default position:", e)
        DEFAULT_X, DEFAULT_Y = 90, 90  # Fallback values
else:
    DEFAULT_X, DEFAULT_Y = 90, 90  # Fallback if no valid response
print(f"Using default position: X={DEFAULT_X}, Y={DEFAULT_Y}")

# === Define Camera and Servo Limits ===
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
# Adjust these limits relative to your default position and physical constraints.
SERVO_X_MIN = DEFAULT_X - 60
SERVO_X_MAX = DEFAULT_X + 60
SERVO_Y_MIN = DEFAULT_Y - 10
SERVO_Y_MAX = DEFAULT_Y + 10

# === Setup Mediapipe for Pose Detection ===
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()
mp_drawing = mp.solutions.drawing_utils

# === Start Webcam ===
cap = cv2.VideoCapture(0)
cap.set(3, FRAME_WIDTH)
cap.set(4, FRAME_HEIGHT)

def map_value(value, in_min, in_max, out_min, out_max):
    """Map a value from one range to another and return an integer."""
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def get_servo_angles(target_x, target_y):
    """Convert target pixel coordinates to servo angles.
       Horizontal mapping is inverted so that moving left in the frame results in left movement.
       Vertical mapping is normal: top of the frame (low y) maps to SERVO_Y_MIN (up) and
       bottom (high y) maps to SERVO_Y_MAX (down).
    """
    # Horizontal: invert mapping.
    servo_x = map_value(target_x, 0, FRAME_WIDTH, SERVO_X_MAX, SERVO_X_MIN)
    # Vertical: normal mapping.
    servo_y = map_value(target_y, 0, FRAME_HEIGHT, SERVO_Y_MIN, SERVO_Y_MAX)
    return servo_x, servo_y

def send_servo_angles(x_angle, y_angle):
    """Send the servo angles to the Arduino after converting to int."""
    if ser.is_open:
        command = f"{int(x_angle)},{int(y_angle)}\n"
        ser.write(command.encode())
        print("Sent:", command.strip())
        time.sleep(0.05)  # Prevent spamming commands

prev_x, prev_y = DEFAULT_X, DEFAULT_Y

print("Press 'q' to exit.")
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame.")
        break

    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = pose.process(frame_rgb)

    if results.pose_landmarks:
        # Use the nose (landmark 0) as the target.
        head_x = results.pose_landmarks.landmark[0].x
        head_y = results.pose_landmarks.landmark[0].y

        # Convert normalized coordinates to pixel values.
        target_x = int(head_x * FRAME_WIDTH)
        target_y = int(head_y * FRAME_HEIGHT)

        # Draw the target for debugging.
        cv2.circle(frame, (target_x, target_y), 10, (0, 255, 0), -1)
        cv2.putText(frame, "Target", (target_x + 10, target_y), cv2.FONT_HERSHEY_SIMPLEX, 
                    0.5, (0, 255, 0), 2, cv2.LINE_AA)

        # Map the target pixel coordinates to servo angles.
        servo_x, servo_y = get_servo_angles(target_x, target_y)
        
        # Only send new commands if there is a change (to reduce jitter).
        if servo_x != prev_x or servo_y != prev_y:
            send_servo_angles(servo_x, servo_y)
            prev_x, prev_y = servo_x, servo_y

    cv2.imshow("Nerf Turret Tracking", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
ser.close()

