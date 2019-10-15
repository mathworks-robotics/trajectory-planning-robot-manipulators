function plotTrajectory(t,q,varargin)
% Plots trajectory given waypoints
% The rules are:
%   a. Each joint/coordinate is plotted in a separate figure window
%   b. If derivatives (qd and qdd) are optionally specified, they will be
%      placed in additional axes of a subplot
%   c. Optional name-value pairs can be used after the above arguments
%
%   Examples:
%   plotTrajectory(t,q)
%   plotTrajectory(t,q,qd,'Names',["ShoulderAngle","ElbowAngle","WristAngle"])
%   plotTrajectory(t,q,qd,qdd,'WaypointTimes',waypointTimes,'Names',["X","Y","Z"])
%
%   Copyright 2019 The MathWorks, Inc.

% Find the order of the trajectory (how many derivatives are provided)
order = 1; % Position only
if nargin > 2 && isnumeric(varargin{1})
    order = 2; % Position and velocity
    if nargin > 3 && isnumeric(varargin{2})
       order = 3; % Position, velocity, and acceleration 
    end
end

% Initialize default plot options
numCoords = size(q,1);
configNames = "Coordinate " + string(1:numCoords); % Coordinate names
waypointTimes = [];

% Now look for additional property-value pairs

if (nargin > order + 1)
    searchIdx = order;
    while searchIdx <= nargin-order
        propName = lower(varargin{searchIdx});
        switch propName
            case 'waypointtimes'    % Waypoint times
                waypointTimes = varargin{searchIdx+1};
            case 'names'            % Set coordinate names
                configNames = varargin{searchIdx+1};
        end
        searchIdx = searchIdx + 2;
    end
end

% Loop through all the coordinates and plot
for idx = 1:numCoords
    figure;
    
    % Always plot time and position only
    subplot(order,1,1), hold on;
    plot(t,q(idx,:));
    plotTimeLines(waypointTimes);
    ylabel('Position');
    title("Trajectory for " + configNames(idx))
    
    % Plot velocity if provided
    if nargin > 2
        subplot(order,1,2), hold on;
        qd = varargin{1};
        plot(t,qd(idx,:));
        plotTimeLines(waypointTimes);
        ylabel('Velocity');
        
            % Plot acceleration if provided
            if nargin > 3
                subplot(order,1,3), hold on;
                qdd = varargin{2};
                plot(t,qdd(idx,:));
                plotTimeLines(waypointTimes);
                ylabel('Acceleration');
            end     
    end
    
    % Finally, label the time axis
    xlabel('Time');
   
end

end

% Helper function to plot vertical lines for the waypoint times
function plotTimeLines(t)
    for idx = 1:numel(t)
       xline(t(idx),'r--'); 
    end
end

