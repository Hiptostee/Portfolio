import cv2
import serial
import time
import numpy as np

# === SERIAL SETUP ===
# arduino_port = '/dev/cu.usbserial-110' 
baud_rate = 115200
# arduino = serial.Serial(arduino_port, baud_rate, timeout=1)
time.sleep(2)

# === VIDEO CAPTURE ===
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"Resolution: {frame_width}x{frame_height}")

# === COLOR RANGE (HSV) ===
# Blue object
lower_color = np.array([100, 150, 50])
upper_color = np.array([140, 255, 255])

# Red - lower range
lower_red1 = np.array([0, 100, 100])
upper_red1 = np.array([10, 255, 255])

# Red - upper range
lower_red2 = np.array([160, 100, 100])
upper_red2 = np.array([179, 255, 255])


lower_yellow = np.array([20, 100, 100])
upper_yellow = np.array([30, 255, 255])

lower_orange = np.array([10, 100, 100])
upper_orange = np.array([20, 255, 255])

lower_purple = np.array([130, 50, 50])
upper_purple = np.array([160, 255, 255])



while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create mask based on color range
    mask = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
    # mask = cv2.inRange(hsv, lower_orange, upper_orange)

    # Morphological operations to clean up noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Largest contour
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) > 500:  # Filter small blobs
            x, y, w, h = cv2.boundingRect(largest)
            center_x = x + w // 2
            center_y = y + h // 2

            # Send data to Arduino
            data = f"{center_x},{center_y},{w},{h}\n"
            # arduino.write(data.encode())

            # Draw rectangle and center
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

    # Show frames
    cv2.imshow('Color Tracker', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
# arduino.close()
cv2.destroyAllWindows()
