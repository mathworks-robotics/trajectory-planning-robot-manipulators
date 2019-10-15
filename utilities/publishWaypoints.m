% Publishes waypoints as ROS messages to test ROS interface
% Copyright 2019 The MathWorks, Inc.

%% Setup
rosshutdown;
rosinit; % You can add your own IP here if using an external ROS master
[wpPub,wpMsg] = rospublisher('/waypoints','geometry_msgs/PoseArray');
[wpTimePub,wpTimeMsg] = rospublisher('/waypoint_times','std_msgs/Float64MultiArray');

%% Publish waypoint data
% NOTE: Requires variables to be defines as in the |createWaypointData| script.
wpMsg.Poses = []; % Clear out any old data
for idx = 1:size(waypoints,2)
    poseMsg = rosmessage('geometry_msgs/Pose');
    poseMsg.Position.X = waypoints(1,idx);
    poseMsg.Position.Y = waypoints(2,idx);
    poseMsg.Position.Z = waypoints(3,idx);
    quaternion = eul2quat(orientations(:,idx)');
    poseMsg.Orientation.W = quaternion(1);
    poseMsg.Orientation.X = quaternion(2);
    poseMsg.Orientation.Y = quaternion(3);
    poseMsg.Orientation.Z = quaternion(4);
    wpMsg.Poses(idx) = poseMsg;
end

% Package and publish the waypoint times
wpTimeMsg.Data = waypointTimes;

% Send the messages
send(wpPub,wpMsg);
send(wpTimePub,wpTimeMsg);