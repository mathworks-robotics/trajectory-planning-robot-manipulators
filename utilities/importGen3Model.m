function importGen3Model
% Imports Kinova Gen3 Ultra lightweight robot model into a Rigid Body Tree
%
% Copyright 2019 The MathWorks, Inc.

    % Gen3 URDF
    addpath(genpath('gen3_description'))
    gen3 = importrobot('JACO3_URDF_V11.urdf');

    % Add a "dummy" gripper link
    gripperLength = 0.1; % Gripper length in meters
    gripperBody = robotics.RigidBody('Gripper');
    gripperJoint = robotics.Joint('GripperLink','fixed');
    T = rotm2tform([0 1 0;0 0 1;1 0 0]) * trvec2tform([gripperLength 0 0]);
    setFixedTransform(gripperJoint,T); % Move and orient the gripper
    gripperBody.Joint = gripperJoint;
    addBody(gen3,gripperBody,'EndEffector_Link');
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