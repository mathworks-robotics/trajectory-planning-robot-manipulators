function visualizeRobot(rbtName,eeName,jointAngles,waypoints,orientations,mode)
% Helper function to visualize rigid body tree in Simulink
% Copyright 2019 The MathWorks, Inc.

% Load rigid body tree and create graphics objects on the first function call
persistent robot hTraj hWaypt ax prevWaypoints prevOrientations
if isempty(robot)
    close all
    figure
    robot = evalin('base',rbtName);
    robot.DataFormat = 'row';
end

% Create graphics objects on the first function call or if waypoints arechanged
if isempty(prevWaypoints) || ~isequal(prevWaypoints,waypoints) || ~isequal(prevOrientations,orientations)
    % Clear the axes
    clf('reset');
    
    % Show the rigid body tree and points again
    ax = show(robot,robot.homeConfiguration,'Frames','off');
    hold on
    title('Robot Trajectory Visualization');
    
    if mode == 2 % Trajectory points
        handTform = getTransform(robot,jointAngles',eeName);
        handPos = tform2trvec(handTform);
        hTraj = plot3(ax,handPos(1),handPos(2),handPos(3),'b.-');
        hWaypt = scatter3(ax,waypoints(1,:),waypoints(2,:),waypoints(3,:),'ro','LineWidth',2);
    end
        
    xlim([-1 1]), ylim([-1 1]), zlim([0 1.2])
    drawnow;
    
    prevWaypoints = waypoints;
    prevOrientations = orientations;
end

% Display the rigid body tree
show(robot,jointAngles','PreservePlot',false,'Frames','off','Parent',ax);
handTform = getTransform(robot,jointAngles',eeName);

% Display the trajectory and waypoint locations
if mode == 2        % Trajectory points
    handPos = tform2trvec(handTform);
    set(hTraj,'XData',[get(hTraj,'XData') handPos(1)], ...
              'YData',[get(hTraj,'YData') handPos(2)], ...
              'ZData',[get(hTraj,'ZData') handPos(3)]);
elseif mode == 3 %   Coordinate frames
    plotTransforms(tform2trvec(handTform),tform2quat(handTform),'FrameSize',0.05); 
end

if ~isscalar(waypoints)
    set(hWaypt,'XData',waypoints(1,:),'YData',waypoints(2,:),'ZData',waypoints(3,:));
end

drawnow;
