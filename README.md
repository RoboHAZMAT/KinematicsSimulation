Kinematics Simulation
=============
RoboHAZMAT: Senior Design Project

Motion Control Team

Gerardo Bledt

October 21, 2014


##**Project Goals:**

The RobotHAZMAT project is a design team made up of 10 Virginia Tech Senior Mechanical Engineers. The primary goal is to develop a method of intuitive gesture control for a robotic system for hazardous response situations. Current hazardous response robots require large amounts of training time and unintuitive button-joystick controls. We hope to increase the range of motion as well as reduces the training time required to operate these systems.

 - This is a first year project serving as a proof of concept that two arm robots can be intuitively controlled through human movement.

 - We hope to be able to develop the robot as well as the wearable user interface using readily available, inexpensive parts.


##**Simulation code Instructions:**

1. In the 'RoboHAZMAT' directory, run 'addpath_RoboHAZMAT.m' to get access to all the Kinematics and utility files needed to run the simulation.

2. In 'RobotSim.m' uncomment lines 52, 57, 62 for arbitrary joint angles for the simulation to show ability to control the motor angles.

3. Run 'RobotSim.m' as 'RobotSim;' to see the simulated RoboHAZMAT robot using predefined joint angles for the robot's DOF. Red markers are movable joints and grey markers are unmovable intersection points.

4. To run the interactive simulation run 'RobotSim(#);' where '#' is any number 1 through 6. However 2, 3, and 6 require the use of the IMU sensor and the Arduino. Try # = 1 first.

5. When prompted, enter y to begin the sim.

6. Next, enter RMK to move the robot's right arm.

7. Give it an x, y, and z location to move the gripper to (try x = 0.35, y = -0.1, z = 0.25)

8. Look at the figure and rotate it around to make sure the right manipulator's last red point is at the specified location.

9. Enter n to quit or y to give another command.

10. When you are done with the test simulation, try the other simulations by running 'RobotSim(#)' and check the 'InteractiveSim.m' file for a description of each simulation.

Code is pretty straight forward... 

Each of the '{Left|Right}ManipulatorKinematics.m' files use the DH parameters and the rotation and translation in the ground frame to give an accurate simulation of the manipulators given thetas for each DOF. Also has the lengths of each link and the physical constraints. All parameters are packaged in a struct and outputed for later use.


 TO DO:

- Current version needs to be cleaned up, documented, and organized.

- Add more commands and interactive simulations for full usage of the  system.

- Add pictures and videos of the full capabilities.
