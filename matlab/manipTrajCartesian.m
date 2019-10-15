% MANIPULATOR TRAJECTORY GENERATION
% Generates Cartesian only (no rotation) trajectories
%
% Copyright 2019 The MathWorks, Inc.

%% Setup
clear, clc, close all

% Define waypoint information
createWaypointData;

% Define IK
ik = inverseKinematics('RigidBodyTree',gen3);
ikWeights = [1 1 1 1 1 1];
ikInitGuess = gen3.homeConfiguration;

% Set up plot
plotMode = 1; % 0 = None, 1 = Trajectory, 2 = Coordinate Frames
show(gen3,jointAnglesHome','Frames','off','PreservePlot',false);
xlim([-1 1]), ylim([-1 1]), zlim([0 1.2])
hold on
if plotMode == 1
    hTraj = plot3(waypoints(1,1),waypoints(2,1),waypoints(3,1),'b.-');
end
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ro','LineWidth',2);

%% Generate trajectory
% Cartesian Motion only
trajType = 'trap';
switch trajType
    case 'trap'
        [q,qd,qdd] = trapveltraj(waypoints,numel(trajTimes), ...
            'AccelTime',repmat(waypointAccelTimes,[3 1]), ... 
            'EndTime',repmat(diff(waypointTimes),[3 1]));
                            
    case 'cubic'
        [q,qd,qdd] = cubicpolytraj(waypoints,waypointTimes,trajTimes, ... 
            'VelocityBoundaryCondition',waypointVels);
        
    case 'quintic'
        [q,qd,qdd] = quinticpolytraj(waypoints,waypointTimes,trajTimes, ... 
            'VelocityBoundaryCondition',waypointVels, ...
            'AccelerationBoundaryCondition',waypointAccels);
        
    case 'bspline'
        ctrlpoints = waypoints; % Can adapt this as needed
        [q,qd,qdd] = bsplinepolytraj(ctrlpoints,waypointTimes([1 end]),trajTimes);
        
    otherwise
        error('Invalid trajectory type! Use ''trap'', ''cubic'', ''quintic'', or ''bspline''');
end

% Show the full trajectory with the rigid body tree
if plotMode == 1
    set(hTraj,'xdata',q(1,:),'ydata',q(2,:),'zdata',q(3,:));
elseif plotMode == 2
    plotTransforms(q',repmat([1 0 0 0],[size(q,2) 1]),'FrameSize',0.05);
end

% To visualize the trajectory, run the following line
% plotTrajectory(trajTimes,q,qd,qdd,'Names',["X","Y","Z"],'WaypointTimes',waypointTimes)

%% Trajectory following loop
for idx = 1:numel(trajTimes) 
    % Solve IK
    tgtPose = trvec2tform(q(:,idx)');
    [config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);
    ikInitGuess = config;

    % Show the robot
    show(gen3,config,'Frames','off','PreservePlot',false);
    title(['Trajectory at t = ' num2str(trajTimes(idx))])
    drawnow    
end