% Compares joint space vs. task space trajectories
% Copyright 2019 The MathWorks, Inc.

%% Setup
clc
createWaypointData;
figure, hold on
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ko:','LineWidth',2);
title('Trajectory Waypoints'); 
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
grid on
view([45 45]);
% Define IK solver
ik = inverseKinematics('RigidBodyTree',gen3);
ikWeights = [1 1 1 1 1 1];
% Use a small sample time for this example, so the difference between joint
% and task space is clear due to evaluation of IK in task space trajectory.
ts = 0.02;
trajTimes = 0:ts:waypointTimes(end);
% Initialize matrices for plots
qTask = zeros(numJoints,numel(trajTimes)); % Derived joint angles in task space trajectory
posJoint = zeros(3,numel(trajTimes)); % Derived end effector positions in joint space trajectory

%% Create and evaluate a task space trajectory
ikInitGuess = jointAnglesHome';
ikInitGuess(ikInitGuess > pi) = ikInitGuess(ikInitGuess > pi) - 2*pi;
ikInitGuess(ikInitGuess < -pi) = ikInitGuess(ikInitGuess < -pi) + 2*pi;

disp('Running task space trajectory generation and evaluation...')
tic;

% Trajectory generation
[posTask,velTask,accelTask] = trapveltraj(waypoints,numel(trajTimes), ...
    'AccelTime',repmat(waypointAccelTimes,[3 1]), ...
    'EndTime',repmat(diff(waypointTimes),[3 1]));

% Trajectory evaluation
for idx = 1:numel(trajTimes) 
    % Solve IK
    tgtPose = trvec2tform(posTask(:,idx)');
    [config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);
    ikInitGuess = config;
    qTask(:,idx) = config;
end

taskTime = toc;
disp(['Task space trajectory time : ' num2str(taskTime) ' s']);

%% Create and evaluate a joint space trajectory
ikInitGuess = jointAnglesHome';
ikInitGuess(ikInitGuess > pi) = ikInitGuess(ikInitGuess > pi) - 2*pi;
ikInitGuess(ikInitGuess < -pi) = ikInitGuess(ikInitGuess < -pi) + 2*pi;

disp('Running joint space trajectory generation and evaluation...')
tic;

% Solve IK for all waypoints
numWaypoints = size(waypoints,2);
numJoints = numel(gen3.homeConfiguration);
jointWaypoints = zeros(numJoints,numWaypoints);
for idx = 1:numWaypoints
    tgtPose = trvec2tform(waypoints(:,idx)');
    [config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);
    cfgDiff = config - ikInitGuess;
    jointWaypoints(:,idx) = config';    
end

% Trajectory Generation
[qJoint,qdJoint,qddJoint] = trapveltraj(jointWaypoints,numel(trajTimes), ...
    'AccelTime',repmat(waypointAccelTimes,[numJoints 1]), ... 
    'EndTime',repmat(diff(waypointTimes),[numJoints 1]));

% Trajectory evaluation (only needed to find end effector position)
for idx = 1:numel(trajTimes)  
    eeTform = getTransform(gen3,qJoint(:,idx)',eeName); 
    posJoint(:,idx) = tform2trvec(eeTform)'; 
end
jointTime = toc;
disp(['Joint space trajectory time : ' num2str(jointTime) ' s']);

%% Create comparison plots
% Compare trajectories in Cartesian space
close all
figure, hold on
plot3(posTask(1,:),posTask(2,:),posTask(3,:),'b-');
plot3(posJoint(1,:),posJoint(2,:),posJoint(3,:),'r--');
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ko','LineWidth',2);
title('Trajectory Comparison'); 
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
legend('Task Space Trajectory','Joint Space Trajectory','Waypoints');
grid on
view([45 45]);

% Compare joint angles
% Plot each joint trajectory
for idx = 1:numJoints
    figure, hold on;
    plot(trajTimes,qTask(idx,:),'b-');
    plot(trajTimes,qJoint(idx,:),'r-');
    for wIdx = 1:numWaypoints
       xline(waypointTimes(wIdx),'k--'); 
    end
    title(['Joint ' num2str(idx) ' Trajectory']); 
    xlabel('Time [s]');
    ylabel('Joint Angle [rad]');
    legend('Task Space Trajectory','Joint Space Trajectory');
end