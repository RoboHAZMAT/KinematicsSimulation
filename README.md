Kinematics Simulation
=============
RoboHAZMAT: Senior Design Project

Motion Control Team

Gerardo Bledt

October 21, 2014

Simulation code 0.1 instructions for the two arms and an arbitrary configuration:

1. In the 'RoboHAZMAT' directory, run 'addpath_RoboHAZMAT.m' to get access to all the Kinematics and utility files needed to run the simulation.

2. In 'RobotSim.m' uncomment lines 12, 17, 22 for arbitrary joint angles for the simulation to show ability to control the motor angles.

3. Run 'RobotSim.m' as 'RobotSim;' to get the full simulation using arbitrary joint angles for each of the DOF within the RoboHAZMAT Robot object's KinematicChains (the left and right manipulators). If this worked it will generate a plot with the T-shaped robot frame body and two arms in an arbitrary configuration. The red markers are joints that can be controlled via 'RoboHAZMAT.KinematicChains.{L|R}MK.thetas.th{1|2|3|4|5|6|7}' where you choose one of the values in the '{}'. The grey markers are cold or rigid joints.

4. To run the interactive simulation (using inverse kinematics to position the arm grippers at the final x,y,z location) run 'RobotSim(1);'

5. When prompted, enter y to begin the sim.

6. Next, enter RMK to move the robot's right arm.

7. Give it an x, y, and z location to move the gripper to (try x = 0.35, y = -0.1, z = 0.25)

8. Look at the figure and rotate it around to make sure the right manipulator's last red point is at the specified location.

9. Enter n to quit or y to give another command.

10. When you are done with the test simulation, try the other simulations by running 'RobotSim(#)' where # is an integer 1 to 6. (2, 3, and 6 require the use if the IMU and 6 requires connection to the Arduino so if you don't have these, try 4 or 5).

Code is pretty straight forward... 

Each of the '{Left|Right}ManipulatorKinematics.m' files use the DH parameters and the rotation and translation in the ground frame to give an accurate simulation of the manipulators given thetas for each DOF. Also has the lengths of each link and the physical constraints. All parameters are packaged in a struct and outputed for later use.

The 'TrajectoryPlanningOptimization' directory is currently being built. Will include the constrained nonliner optimization code. That will allow the user to specify an (x,y,z) location to put one of the manipulators in and it will find the best possible configuration to get there if it is a valid point to reach.


 TO DO:

- Current version needs to be cleaned up, documented, and organized.

- The interactive simulation needs to be fully completed... allows the user to input commands and have the robot execute the commands.

- Work on communicating ith Arduino to input commands remotely.
