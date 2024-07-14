import cv2
import numpy as np
import math
import time
import robomaster
from robomaster import robot
import keyboard

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False)
    
    chassis = ep_robot.chassis
    pid_controller = PIDController(kp=1.8, ki=0.2, kd=0.2)

    while True:
        start_time = time.time()
        frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)

        roi = frame[frame.shape[0] // 3:, :]

        # Convert the ROI to HSV color space
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Define range for green color in HSV
        lower_green = np.array([35, 100, 100])
        upper_green = np.array([85, 255, 255])

        # Threshold the HSV image to get only green colors
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Apply the mask to the original ROI
        masked_image = cv2.bitwise_and(roi, roi, mask=mask)

        # Convert the masked image to grayscale
        gray = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)

        # Blur the grayscale image to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Find the edges in the image using Canny detector
        threshold1 = 50
        threshold2 = 150
        edged = cv2.Canny(blurred, threshold1, threshold2)

        # Detect lines using Hough transform
        lines = cv2.HoughLinesP(edged, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)

        theta = 0
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(roi, (x1, y1), (x2, y2), (255, 0, 0), 3)
                theta += math.atan2((y2 - y1), (x2 - x1))

            angle = np.degrees(theta)

            error = angle
            dt = time.time() - start_time
            angular_velocity = pid_controller.update(error, dt)
            angular_velocity = np.clip(angular_velocity, -30, 30)

            chassis.drive_speed(x=0.5, y=0, z=angular_velocity, timeout=0.1)

        cv2.imshow("Edges", edged)
        cv2.imshow("Image", roi)
        cv2.waitKey(1)

        if keyboard.is_pressed('q'):
            break

    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_robot.close()
