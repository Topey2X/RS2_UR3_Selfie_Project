% Initialize ROS
rosinit;

% Create a subscriber for the optimized path topic
optimizedPathSub = rossubscriber('/optimized_path', 'nav_msgs/Path');

% Create a figure for plotting
figure;
hold on;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Optimized Trajectory');

% Initialize x, y, and z to empty arrays
x = [];
y = [];
z = [];

% Loop to receive data until x, y, and z are non-zero
while all(x == 0) && all(y == 0) && all(z == 0)
    % Wait for data to be available
    while optimizedPathSub.LatestMessage == 0
        pause(0.1); % Pause for a short duration to avoid consuming too much CPU
    end
    
    % Receive the latest message from the subscriber
    pathMsg = receive(optimizedPathSub, 20); % Timeout of 20 seconds
    
    % Extract the positions from the received message
    positions = pathMsg.Poses;
    
    % Extract the x, y, and z coordinates from the positions
    x = arrayfun(@(pose) pose.Pose.Position.X, positions);
    y = arrayfun(@(pose) pose.Pose.Position.Y, positions);
    z = arrayfun(@(pose) pose.Pose.Position.Z, positions);
    
    % Print the dimensions of x, y, and z arrays
    fprintf('Dimensions of x: %s\n', mat2str(size(x)));
    fprintf('Dimensions of y: %s\n', mat2str(size(y)));
    fprintf('Dimensions of z: %s\n', mat2str(size(z)));
end

fprintf("Completed Extracting Data");

% Set the axis limits based on the data range
xlim([min(x)-100, max(x)+100]); % Add a bit of padding
ylim([min(y)-100, max(y)+100]);
zlim([min(z)-1, max(z)+1]);

view(180,90);

% Set up the animation
numFrames = length(x);
trajAnimation = animatedline('Color', 'b', 'LineWidth', 2);

% Create the animation
for i = 1:numFrames
    addpoints(trajAnimation, x(i), y(i), z(i));
    drawnow;
    pause(0.01); % Adjust the pause duration to control the animation speed
end

legend('Optimized Trajectory');


pause(20);
rosshutdown