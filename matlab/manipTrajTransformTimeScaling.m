% MANIPULATOR TRAJECTORY GENERATION
% Generates combined transform (rotation and translation) trajectories 
% using custom time scaling from a separate trajectory.
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
plotMode = 2; % 0 = None, 1 = Trajectory, 2 = Coordinate Frames
show(gen3,gen3.homeConfiguration,'Frames','off','PreservePlot',false);
xlim([-1 1]), ylim([-1 1]), zlim([0 1.2])
hold on
if plotMode == 1
    hTraj = plot3(waypoints(1,1),waypoints(2,1),waypoints(3,1),'b.-');
end
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ro','LineWidth',2);

%% Generate and follow trajectory
% Loop through segments one at a time
trajType = 'trap';
numWaypoints = size(waypoints,2);
for w = 1:numWaypoints-1
        
    % Get the initial and final transforms and times for the segment
    T0 = trvec2tform(waypoints(:,w)') * eul2tform(orientations(:,w)');
    Tf = trvec2tform(waypoints(:,w+1)') * eul2tform(orientations(:,w+1)');
    timeInterval = waypointTimes(w:w+1);
    trajTimes = timeInterval(1):ts:timeInterval(2);
    
    % Generate time scaling trajectory for the segment on the range [0 1]
    switch trajType
        case 'trap'
            [s,sd,sdd] = trapveltraj([0 1],numel(trajTimes), ... 
                                     'EndTime',diff(timeInterval));
        case 'cubic'
            [s,sd,sdd] = cubicpolytraj([0 1],timeInterval,trajTimes);
        case 'quintic'
            [s,sd,sdd] = quinticpolytraj([0 1],timeInterval,trajTimes);
        otherwise
            error('Invalid trajectory type! Use ''trap'', ''cubic'', or ''quintic''');
    end
    
    % Find the transforms from trajectory generation
    [T,vel,acc] = transformtraj(T0,Tf,timeInterval,trajTimes, ... 
                                'TimeScaling',[s;sd;sdd]);  
       
    % Trajectory visualization for the segment
    if plotMode == 1
        eePos = tform2trvec(T);
        set(hTraj,'xdata',eePos(:,1),'ydata',eePos(:,2),'zdata',eePos(:,3));
    elseif plotMode == 2
        plotTransforms(tform2trvec(T),tform2quat(T),'FrameSize',0.05)
    end
    
    % Trajectory following for the segment
    for idx = 1:numel(trajTimes) 
        % Solve IK
        tgtPose = T(:,:,idx);
        [config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);
        ikInitGuess = config;

        % Show the robot
        show(gen3,config,'Frames','off','PreservePlot',false);
        title(['Trajectory at t = ' num2str(trajTimes(idx))])
        drawnow
    end
    
end