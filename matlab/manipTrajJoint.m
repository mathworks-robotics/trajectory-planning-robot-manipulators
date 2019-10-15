% MANIPULATOR TRAJECTORY GENERATION 
% Generates Joint space trajectories by performing inverse kinematics on
% each waypoint and interpolating between the joint angles.
%
% Copyright 2019 The MathWorks, Inc.

%% Setup
clear, clc, close all

% Define waypoint information
createWaypointData;

% Define IK
ik = inverseKinematics('RigidBodyTree',gen3);
ikWeights = [1 1 1 1 1 1];
ikInitGuess = jointAnglesHome';
ikInitGuess(ikInitGuess > pi) = ikInitGuess(ikInitGuess > pi) - 2*pi;
ikInitGuess(ikInitGuess < -pi) = ikInitGuess(ikInitGuess < -pi) + 2*pi;

% Set up plot
plotMode = 1; % 0 = None, 1 = Trajectory, 2 = Coordinate Frames
show(gen3,gen3.homeConfiguration,'Frames','off','PreservePlot',false);
xlim([-1 1]), ylim([-1 1]), zlim([0 1.2])
hold on
if plotMode == 1
    hTraj = plot3(waypoints(1,1),waypoints(2,1),waypoints(3,1),'b.-');
end
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ro','LineWidth',2);

%% Solve IK for all waypoints
includeOrientation = false; % Set this to use zero vs. nonzero orientations

numWaypoints = size(waypoints,2);
numJoints = numel(gen3.homeConfiguration);
jointWaypoints = zeros(numJoints,numWaypoints);

for idx = 1:numWaypoints
    if includeOrientation
        tgtPose = trvec2tform(waypoints(:,idx)') * eul2tform(orientations(:,idx)');
    else
        tgtPose =  trvec2tform(waypoints(:,idx)');
    end
    [config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);
    jointWaypoints(:,idx) = config';
end

%% Generate trajectory on joint space
trajType = 'trap';
switch trajType
    case 'trap'
        [q,qd,qdd] = trapveltraj(jointWaypoints,numel(trajTimes), ...
            'AccelTime',repmat(waypointAccelTimes,[numJoints 1]), ... 
            'EndTime',repmat(diff(waypointTimes),[numJoints 1]));
                            
    case 'cubic'
        [q,qd,qdd] = cubicpolytraj(jointWaypoints,waypointTimes,trajTimes, ... 
            'VelocityBoundaryCondition',zeros(numJoints,numWaypoints));
        
    case 'quintic'
        [q,qd,qdd] = quinticpolytraj(jointWaypoints,waypointTimes,trajTimes, ... 
            'VelocityBoundaryCondition',zeros(numJoints,numWaypoints), ...
            'AccelerationBoundaryCondition',zeros(numJoints,numWaypoints));
        
    case 'bspline'
        ctrlpoints = jointWaypoints; % Can adapt this as needed
        [q,qd,qdd] = bsplinepolytraj(ctrlpoints,waypointTimes([1 end]),trajTimes);
        
    otherwise
        error('Invalid trajectory type! Use ''trap'', ''cubic'', ''quintic'', or ''bspline''');
end

% To visualize the trajectory, run the following line
% plotTrajectory(trajTimes,q,qd,qdd,'Names',"Joint " + string(1:numJoints),'WaypointTimes',waypointTimes)

%% Trajectory following loop
for idx = 1:numel(trajTimes)  

    config = q(:,idx)';
    
    % Find Cartesian points for visualization
    eeTform = getTransform(gen3,config,eeName);
    if plotMode == 1
        eePos = tform2trvec(eeTform);
        set(hTraj,'xdata',[hTraj.XData eePos(1)], ...
                  'ydata',[hTraj.YData eePos(2)], ...
                  'zdata',[hTraj.ZData eePos(3)]);
    elseif plotMode == 2
        plotTransforms(tform2trvec(eeTform),tform2quat(eeTform),'FrameSize',0.05);
    end

    % Show the robot
    show(gen3,config,'Frames','off','PreservePlot',false);
    title(['Trajectory at t = ' num2str(trajTimes(idx))])
    drawnow   
    
end