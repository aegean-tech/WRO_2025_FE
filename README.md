# Mechanics 
Our robot has 3 floors that are independent from each other and each one has a 
unique functionality. These floors called “Lower Part”, “Middle Part” and “Upper Part”. 
For the floors we used plexi material. In this section we are going to illustrate these 
parts and their functionalities. The general of the robot length is 25 and our robots 
width is 18. Also our robot weighs 1320g and passes the rule.

 
## 1. Lower Part Of the Robot 
In the lower part of the chassis, we integrate the essential mechanical 
systems responsible for the robot’s movement, including the drive system, 
steering system, and the LiPo battery that powers the drive hub. This 
section is designed to provide structural stability while ensuring that the 
components responsible for motion are securely housed and properly aligned 
with the rest of the robot’s framework. 
Our drive system is built around two yellow rubber wheels, each with a 
diameter of 65mm and a weight of 38 grams. These wheels are 
carefully selected for their balance between traction and size, making them 
suitable for both speed and maneuverability on different surfaces. Both 
wheels are mounted onto a single shaft, allowing synchronized rotation, which 
is crucial for maintaining directional stability during forward or backward 
movement. 

The motor we use is a 12V, 35mm, 960 RPM DC Gearmotor, 
chosen for its balance of torque and speed. Its built-in gearbox allows us to 
harness high torque at a reasonable rotation speed, which is essential for 
moving the robot with both force and control, especially when carrying 
additional components on the upper floors. This motor is mounted securely to 
the chassis using a custom bracket to prevent vibration and misalignment, 
which could affect the gear connection and wheel performance. 

We use a Profuse LiPo battery to power our motor controller. It 
provides stable and sufficient energy for the drive system. An XT60 
connector is attached to ensure a secure and efficient power connection with 
minimal loss. 


## 2.  Middle Part Of the Robot 
In the middle part of the chassis, we house several critical electronic and 
control components that serve as the brain and sensory hub of the robot. This 
section includes the microcontroller, steering servo, main power switch, 
and ultrasonic sensors, all strategically placed for optimal performance and 
accessibility during maintenance or upgrades. 
At the heart of this layer lies the Raspberry Pi 5 with 8GB RAM, 
which functions as the robot’s main microcontroller. The Raspberry Pi is 
responsible for executing all high-level software operations, such as 
processing sensor inputs, controlling the movement systems, handling 
camera data, and managing the overall workflow of the robot. Its processing 
power and memory capacity make it suitable for running multiple scripts 
simultaneously and handling real-time operations efficiently. 


The steering servo, which controls the front wheels' angle, is also integrated 
into the middle part of the chassis. Although the actual turning mechanism is 
located on the lower part of the robot, the servo motor itself is mounted on the 
middle layer to save space and maintain structural integrity. To connect the 
servo to the steering system, we designed and 3D printed a custom shaft 
tunnel — a precisely aligned hole through the middle floor — which allows the 
servo shaft to reach down to the steering assembly on the lower level.
This design effectively transfers the servo’s motion while keeping the 
motor safely positioned and reducing stress on the structure. 
In addition to these, we included a physical switch in the middle floor to act 
as a main power control unit. This switch is wired directly between the LiPo 
battery and the motor controller, enabling us to safely start or shut down the 
robot without disconnecting any wires. This not only enhances safety during 
operation but also simplifies troubleshooting and testing during development 
phases. 

Another essential part of the middle section is the ultrasonic sensor system. 
We mounted three HC-SR04 ultrasonic sensors to allow the robot to 
perceive and interact with its environment. One sensor is placed facing 
forward to detect obstacles in the robot’s path, while the other two are 
positioned on the left and right sides to monitor lateral distances. These 
sensors are mounted securely using 3D-printed custom beds that are 
designed to fit the robot’s structure and keep the sensors stable and 
accurately aligned. These mounts also provide easy removal and 
replacement, making the system modular and upgrade-friendly. 
Together, the components on the middle floor form the core of the robot’s 
decision-making and navigation systems. Their layout was carefully planned 
to ensure compactness, accessibility, and efficient cable management all 
of which contribute to a robust and professional robotic platform.

## 3. Upper Part Of the Robot 
This part of the robot is smaller than the other floors of the robot. In this part 
we have the motor controller and our Raspberry Pi Module V3 
camera on the upper part of the robot.  
 
# Software 

We use Raspberry Pi OS for coding our components. Using a micro sd card for 
uploading a OS to Raspberry and connecting a micro HDMI cable to use its 
interface. Our software developes by two parts. First part we have robot 
functionalities and second part is to detect boxes using camera module. In our code 
each component has own file. Then we gathering all the useful methods and perform 
the continuous sequence in the main file. 

## 1. Robot Functionalities 
Our robot’s movement functionalities are implemented across three separate 
files, each dedicated to controlling a specific component of the robot. The first 
file manages the servo motor functionalities. In this file, the servo motor’s 
PWM port is configured, and its movements are controlled accordingly. A total 
of four functions are defined for the servo. The set_angle(angle) function 
allows precise adjustment of the servo to the desired angle. Additionally, three 
directional functions facilitate steering: steer_left() sets the servo to 60 
degrees to turn the robot left, steer_center() positions the servo at 90 degrees, 
representing the center or neutral position for straight movement, and 
steer_right() moves the servo to 120 degrees to turn right. These functions 
enable accurate and straightforward control of the robot’s steering based on 
the servo motor’s position. 

The second file handles the ultrasonic sensor code. This Python script utilizes 
the lgpio library to interface with ultrasonic distance sensors on a Raspberry 
Pi or similar Linux-based systems. It defines a class named UltrasonikSensor, 
which configures and operates each sensor by assigning specific GPIO pins 
for trigger (TRIG) and echo (ECHO) functions. The class’s olc method sends a 
short pulse from the TRIG pin and listens for its reflection on the ECHO pin, 
measuring the time delay to calculate the distance using the speed of sound. 
Three sensors are initialized to measure distances at the front, right, and left 
sides of the robot. The program continuously reads and displays these 
measurements every second, making it suitable for obstacle detection in 
robotic applications. It should be noted that there is a typographical error in 
the class constructor, where init is used instead of the correct __init__, which 
must be corrected for the class to function properly. 

The third file contains the drive motor control code. This code specifies the 
GPIO pins used for the right PWM (RPWM), left PWM (LPWM), left enable 
(L_EN), and right enable (R_EN) signals. Four primary functions are defined 
to control the robot’s movement: forward(), backward(), stop(), and cleanup(). 
The forward() function powers the motors at a speed value of 200 to move the 
robot forward, with appropriate HIGH and LOW signals to set the motor 
direction. The backward() function similarly sets the motors to move backward 
at the same speed. The stop() function halts the motors without requiring any 
speed parameters. Finally, the cleanup() function is invoked at the start and 
end of the program to properly reset the GPIO pins and prevent conflicts or 
resource leaks. 

Together, these three files provide coordinated control over the robot’s 
mobility and environmental awareness. The servo motors enable precise 
steering, ultrasonic sensors detect obstacles in the robot’s surroundings, and 
the drive motors facilitate safe and controlled movement. Ensuring the 
correctness and efficiency of these codes is critical for achieving reliable and 
effective robot performance. 

## 2. Detecting Boxes 
For detecting and sorting boxes, our robot relies on a color detection system, 
which enables it to identify boxes based on their distinct colors and respond 
accordingly. The system is implemented in two separate Python files: one 
dedicated to calibrating the colors of the boxes, and the other responsible for 
performing real-time sorting actions based on the live camera feed. 
The color detection process is divided into two main stages: calibration and 
real-time sorting. 

During calibration, we ensure that the environment is consistently lit to avoid 
discrepancies caused by lighting conditions. We place each box under the 
camera's view, making sure that shadows and reflections are minimized. The 
user then clicks on a specific region of the box within the camera frame, and 
the software captures the color data at that pixel. This data includes HSV 
(Hue, Saturation, Value) or RGB (Red, Green, Blue) values, which are 
essential for accurate color detection. 
These sampled color values are stored in a .json configuration file. This file 
acts as a reference for the real-time sorting script, allowing the robot to 
identify the boxes by comparing their detected colors with the calibrated 
values. This method ensures consistent and accurate detection, even across 
multiple sessions. 

Once calibration is completed, the real-time detection system becomes active. 
The camera continuously analyzes the incoming video feed to search for 
boxes that match the calibrated color profiles. When a box is detected, a 
bounding box is drawn around it for visual confirmation, and a small dot is 
marked at the center of the detected box for tracking purposes. 
To determine the appropriate movement, the screen is virtually divided into 
three equal vertical sections: left, center, and right. Depending on where the 
center dot of the detected box lies, the robot takes the necessary action to 
align itself with the box. 
For example, if a green box is detected and its center lies in the right section 
of the screen, the robot turns left until the box is aligned with the center or left 
section. Once the box reaches the target area, the robot stabilizes itself and 
continues forward, effectively completing the sorting process. 
This approach not only allows precise interaction with the environment but 
also gives us the flexibility to handle boxes of different colors and positions in 
real time. Thanks to the modular structure of the code, additional colors or 
actions can easily be integrated into the system in future versions. 

Prototyping Process and Improvements 
Throughout the development process, we went through several prototyping stages to 
arrive at a functional and efficient final robot design. One of the key components we 
focused on was the steering system. Initially, we experimented with placing the servo 
motor responsible for steering on the lowest floor of the robot. However, this led to 
an imbalance between the front and back wheels due to the height difference. To 
address this, we relocated the servo to the middle floor and used a custom 3D
printed structure to transfer the servo shaft down to the steering mechanism on the 
lower level. This approach not only resolved the height discrepancy but also resulted 
in a smooth and precise steering system that aligned well with competition 
standards. 

During this prototyping phase, we also considered using a downward-facing color 
sensor to detect floor colors, such as orange and blue, to guide the robot’s 
directional decisions. The idea was to let the robot decide whether to go left or right 
based on the color it detected underneath. While this method showed some initial 
promise, it was ultimately replaced by a more reliable and adaptable solution 
involving ultrasonic sensors. By placing sensors on both sides of the robot, we could 
compare the distance readings. If the right-side ultrasonic detected a significantly 
greater distance than the left-side sensor, it indicated that the robot had moved 
closer to the left and needed to correct by turning right. This method proved more 
consistent under varying lighting conditions and floor textures. 
For the drive system, we tested two major prototypes. In the first design, we 
attempted to directly couple the motor and wheel shafts without any gears. This 
approach introduced many mechanical challenges, including poor alignment and 
limited control over speed or torque. Despite multiple attempts and adjustments, we 
couldn't achieve a stable and effective connection. 

In the second design iteration, we decided to incorporate connecting gears between 
the motor and the wheel shaft. Initially, we planned to place the motor underneath 
the floor and align the gear system beneath the plexiglass. However, this setup 
faced a new challenge: the motor's vertical height did not fit within the space 
between the floors. To overcome this, we reoriented the motor horizontally and 
routed the gear system on top of the plexiglass surface. This adjustment allowed the 
wheel shaft to align perfectly with the front wheels in terms of height, maintaining 
structural symmetry. Moreover, the gear-based configuration granted us the flexibility 
to modify gear ratios, letting us fine-tune the system for either higher torque or 
increased speed based on the requirements of the match. 

One of the significant software improvements we implemented was the addition of a 
calibration mode for color detection. Initially, we faced challenges in reliably 
identifying the colors of boxes due to varying lighting conditions, which affected the 
camera's performance. To resolve this, we developed a dedicated calibration 
command. This command allows the robot to dynamically recalibrate its color 
thresholds before operation. As described in the Software section, this mode greatly 
increased the accuracy and consistency of our real-time color-based box detection 
and sorting system.
