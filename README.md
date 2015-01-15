Kinematics Simulation
=============
RoboHAZMAT: Senior Design Project

Motion Control Team

Gerardo Bledt

October 21, 2014

#####Click here for video: [Prototype Demo!](https://www.youtube.com/watch?v=7mTqpFiKheA&feature=youtu.be)

##**Project Goals:**

The RobotHAZMAT project is a design team made up of 10 Virginia Tech Senior Mechanical Engineers. The primary goal is to develop a method of intuitive gesture control for a robotic system for hazardous response situations. Current hazardous response robots require large amounts of training time and unintuitive button-joystick controls. We hope to increase the range of motion as well as reduces the training time required to operate these systems.

 - This is a first year project serving as a proof of concept that two arm robots can be intuitively controlled through human movement.

 - We hope to be able to develop the robot as well as the wearable user interface using readily available, inexpensive parts.


##**Basic Simulation Instructions:**

 1. In the 'RoboHAZMAT' directory, run 'addpath_RoboHAZMAT.m' to get access to all the directories needed to run the simulation.

 3. Run 'RobotSim.m' as 'RobotSim;' to see the defalut simulation of the RoboHAZMAT robot using predefined joint angles for the robot's DOF. Red markers are movable joints and grey markers are unmovable intersection points. 

 4. To run different interactive simulations, run 'RobotSim(#);' where '#' is a number {1,2,3,4,5,11,12,13,14,15}. However {4,5,13,14,15} require the use of the IMU sensor and / or the Arduino. {1-5} deal with the RoboHAZMAT robot and {11-15} deal with the Mechatronic Arm.

 5. When prompted, enter 'y' to begin the simulation.

 6. Follow the instructions on the MATLAB command window to interact with the simulation. Make sure to enter the commands when the Figure window is active (click on it to activate) and that no pan, zoom, or rotate options are currently selected. The only exception is simulation 2, where commands are entered in the command window.

 7. To quit press space and then in the command window press 'y' to continue the simulation or 'n' to quit.

 8. When you are done with the test simulation, try the other simulations by running 'RobotSim(#)' and check the 'InteractiveSim.m' script in the Simulations directory for a description of each simulation.


 ##**IMU Controlled Simulation Instructions:**
 
 1. In the 'RoboHAZMAT' directory, run 'addpath_RoboHAZMAT.m' to get access to all the directories needed to run the simulation.

 2. Upload the 'IMUQuat.ino' Arduino sketch through USB onto the Arduino that is attached to an MPU9150 IMU unit.

 3. Check the COM port that corresponds to the Arduino by clicking on: Windows Start Menu -> Control Panel -> Device Manager -> Ports

 4. Open the 'SetupCOM.m' script and change the IMUCOM array to match the COM ports found in step 3.

 5. Run 'RobotSim(#);' where '#' is a number {1,2,3,4,5,11,12,13,14,15}. However {13,14,15} require the use of the Arduino. {1-5} deal with the RoboHAZMAT robot and {11-15} deal with the Mechatronic Arm.
 
 6. The IMUs will be calibrated so make sure that they are in a position where they will not move until ready. When prompted, enter 'y' to begin the simulation.

 6. Hold your arm straight out infront of you when beginning control. 

 7. Follow the instructions on the MATLAB command window to interact with the simulation. Make sure to enter the commands when the Figure window is active (click on it to activate) and that no pan, zoom, or rotate options are currently selected. The only exception is simulation 2, where commands are entered in the command window.

 8. To quit press space and then in the command window press 'y' to continue the simulation or 'n' to quit.

 9. When you are done with the test simulation, try the other simulations by running 'RobotSim(#)' and check the 'InteractiveSim.m' script in the Simulations directory for a description of each simulation.


Each of the '{Left|Right}ManipulatorKinematics.m' files use the DH parameters and the rotation and translation in the ground frame to give an accurate simulation of the manipulators given thetas for each DOF. Also has the lengths of each link and the physical constraints. All parameters are packaged in a struct and outputed for later use.

 TO DO:

- Current version needs to be cleaned up, documented, and organized.

- Add more commands and interactive simulations for full usage of the system.

- Add pictures and videos of the full capabilities.
