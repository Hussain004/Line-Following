<h1> Line Following using Canny Edge Detection and Hough Transform </h1>

![showcase](showcase.png)

<h2> Overview </h2>
This project demonstrates a line following algorithm using computer vision techniques on the DJI Robomaster EP Core robot platform. The robot is equipped with a camera that captures the video feed, which is then processed to detect the presence of lines and determine the appropriate steering angle to follow the line.

<h2> Key Features </h2>

<h3> Color-based Line Detection: </h2>
The robot uses Canny edge detection and Hough transform to identify lines in the camera's field of view. Different color masks are applied to detect lines of various colors, such as yellow, red, orange, green, and blue.

<h3> PID Controller: </h3>
A PID (Proportional-Integral-Derivative) controller is implemented to calculate the angular velocity needed to steer the robot and follow the detected line. The PID coefficients can be adjusted to optimize the robot's responsiveness and stability.

<h3> Real-time Visualization: </h3>
The processed video frames, including the detected edges and lines, are displayed in real-time using OpenCV's built-in display functions. This allows for visual feedback and debugging during the line following process.

<h3> Keyboard Control: </h3>
The user can interrupt the line following process by pressing the 'q' key on the keyboard, which will stop the video stream and close the robot connection.

<h2> Getting Started </h2>
To run this project, you will need the following:

1. A DJI Robomaster EP Core robot.
2. Python 3.x installed on your system.
3. The following Python libraries installed:
    - OpenCV (cv2)
    - NumPy (numpy)
    - Robomaster SDK (robomaster)
    - Keyboard monitoring (keyboard)

Once you have the necessary setup, follow these steps:

1. Clone the repository containing the project files.
2. Ensure that the robot is connected to the same network as your computer.
3. Run the Python script for the desired color line detection (e.g., Yellow.py, Red.py, Orange.py, Green.py, Blue.py).
4. The robot will start the line following process, and the video feed with the detected lines will be displayed.
5. Press 'q' on the keyboard to stop the program.

<h2> Project Structure </h2>
The project consists of the following Python files:

- Yellow.py
- Red.py
- Orange.py
- Green.py
- Blue.py

Each file contains the same core functionality, with the only difference being the color mask used for line detection.

<h2> Customization </h2>
You can customize the project by modifying the following parameters:

<h3> PID Controller Coefficients: </h3>
Adjust the kp, ki, and kd values in the PIDController class to optimize the robot's line following performance.

<h3> Color Mask Range: </h3>
Update the lower_* and upper_* values in the color detection section to adjust the range of the color mask.

<h3> Canny Edge Detection Thresholds: </h3>
Modify the threshold1 and threshold2 values in the edge detection part to improve the line detection accuracy.

<h3> Hough Transform Parameters: </h3>
Adjust the parameters for the cv2.HoughLinesP() function, such as threshold, minLineLength, and maxLineGap, to fine-tune the line detection.