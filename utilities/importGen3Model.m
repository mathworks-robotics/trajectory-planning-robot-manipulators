function importGen3Model
% Imports Kinova Gen3 Ultra lightweight robot model into a Rigid Body Tree
%
% Copyright 2019 The MathWorks, Inc.

    % Load the URDF file
    %
    % NOTE: This requires you to have the kortex_description folder on 
    % your path, which you can download from 
    % https://github.com/Kinovarobotics/ros_kortex.git
    %
    % Then, you have to convert the gen3.xacro file to a URDF file using
    % the following commands in a ROS enabled terminal:
    %  $ cd PATH/TO/ros_kortex/kortex_description/robots
    %  $ rosrun xacro xacro --inorder -o gen3.urdf gen3.xacro
    addpath(genpath('kortex_description'))
    gen3 = importrobot('gen3.urdf');

    % Add a "dummy" gripper link
    gripperLength = 0.1; % Gripper length in meters
    gripperBody = rigidBody('Gripper');
    gripperJoint = rigidBodyJoint('GripperLink','fixed');
    T = rotm2tform([0 1 0;0 0 1;1 0 0]) * trvec2tform([gripperLength 0 0]);
    setFixedTransform(gripperJoint,T); % Move and orient the gripper
    gripperBody.Joint = gripperJoint;
    addBody(gen3,gripperBody,'end_effector_link');
    % Add a "dummy" mesh
    addVisual(gen3.Bodies{9},'Mesh','cylinder.stl', ... 
              trvec2tform([-0.1 0 0]) * axang2tform([0 1 0 pi/2]));
        
    % Configure the data format and show the robot in the home position
    gen3.DataFormat = 'row';
    load gen3positions
    show(gen3,jointAnglesHome');

    % Save the Rigid Body Tree to a file
    curDir = pwd;
    saveDir = fileparts(mfilename('fullpath'));
    cd(saveDir)
    save gen3 gen3
    cd(curDir)

end