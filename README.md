## Trajectory Planning of Robot Manipulators with MATLAB and Simulink
Copyright 2019 The MathWorks, Inc.

## Description

This submission consists of educational MATLAB and Simulink examples for 
trajectory generation and evaluation of robot manipulators. 

All examples feature the 7-DOF Kinova Gen3 Ultra lightweight robotic manipulator: 
https://www.kinovarobotics.com/en/products/robotic-arms/gen3-ultra-lightweight-robot

There is a presaved MATLAB rigid body tree model of the Kinova Gen3; however, you can
access the 3D model description from the Kinova Kortex GitHub repository:
https://github.com/Kinovarobotics/ros_kortex

For more information on the Robotics System Toolbox functionality for manipulators, 
see the documentation: https://www.mathworks.com/help/robotics/manipulators.html

For more background information on trajectory planning, refer to this presentation:
https://cw.fel.cvut.cz/old/_media/courses/a3m33iro/080manipulatortrajectoryplanning.pdf

If you have any questions, email us at roboticsarena@mathworks.com.

## Files
To get started, run the `startupExample.m` script. This will configure the MATLAB search path so all the examples run correctly.

### `matlab` Folder
Contains MATLAB examples for trajectory planning.

* `manipTrajCartesian.m` - Task space (translation only) trajectories
* `manipTrajJoint.m` - Joint space trajectories. Contains an `includeOrientation` variable to toggle waypoint orientations on or off. 
* `manipTrajLinearRotation.m` - Task space (translation only) trajectories with linearly interpolated orientation
* `manipTrajTransform.m` - Linearly interpolated transform trajectories (translation and orientation) 
* `manipTrajTransformTimeScaling.m` - Transform trajectories (translation and orientation) interpolated using nonlinear time scaling
* `compareTaskVsJointTraj.m` - Comparison script that illustrates the difference between task space and joint space trajectories

**NOTE:** All the scripts above are configurable:
* `createWaypointData.m` script - Generates sample waypoints, trajectory times, and other necessary planning variables.
* `trajType` variable - Used to switch the trajectory type
* `plotMode` variable - Used to switch the waypoint/trajectory visualization type

### `simulink` Folder
Contains Simulink examples for trajectory planning.

* `manipCartesianTrajectory.slx` - Task space (translation only) trajectories
* `manipJointTrajectory.slx` - Joint space trajectories.
* `manipRotationTrajectory.slx` - Task space (translation only) trajectories with linearly interpolated orientation
* `manipTransformTrajectory.slx` - Linearly interpolated transform trajectories (translation and orientation) 
* `manipTransformTrajectoryTimeScaling.slx` - Transform trajectories (translation and orientation) interpolated using nonlinear time scaling

**NOTE:** There are also models that work with Robot Operating System (ROS), which are identically named with the `ros` prefix.
Instead of using variables in the MATLAB base workspace, waypoint information is communicated using ROS messages. 
To test this, you can use the Waypoint Publisher App or the `publishWaypoints` script (see the next section).

The ROS topics and message types are:
* `/waypoints` - List of waypoints, message type `geometry_msgs/PoseArray`
* `/waypoint_times` - List of waypoint target times, message type `std_msgs/Float64MultiArray`

### `utilities` Folder
Contains several utilities for the MATLAB and Simulink examples above.

* `createWaypointData.m` - Creates sample waypoints, waypoint times, and other necessary planning variables. If you want to change the waypoints or other trajectory reference values, modify this script (or we suggest creating a copy)
* `cylinder.stl` - "Dummy" mesh file representing the end effector attached the arm
* `gen3.mat` - Presaved rigid body tree containing the 3D model of the robot arm
* `gen3positions.mat` - Presaved joint and end effector configurations for the "home" and "retract" positions of the robot arm
* `importGen3Model.m` - Function to import the Kinova Gen3 manipulator model. Not needed by this example; you can use this if you want to import a new model yourself from the source URDF file.
* `plotTrajectory.m` - Utility function to plot generated trajectory profiles (used with MATLAB examples)
* `publishWaypoints.m` - Tests the publishing of waypoint information as ROS messages
* `trajExampleUtils.slx` - Block library containing common components for the Simulink examples
* `visualizeRobot.m` - Utility function used by the library above to plot the manipulator from a Simulink model
* `waypointPublisher.mlapp` - MATLAB app used to modify waypoints and publish them to the base workspace or as ROS messages