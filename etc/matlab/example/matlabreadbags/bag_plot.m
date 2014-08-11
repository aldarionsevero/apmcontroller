clear rosbag_wrapper;
clear ros.Bag;
clear all
choice=1;

%% Load a bag and get information about it
% Using load() lets you auto-complete filepaths.
bag = ros.Bag.load('matlabsetpoint.bag');
bag.info()

%% Read all messages on a few topics
topic1 = 'filtered_vel';
topic2 = 'filtered_pose';
msgs = bag.readAll({topic1, topic2});

fprintf('Read %i messages\n', length(msgs));

%% Re-read msgs on topic1 and get their metadata
if choice==1

    [msgs, meta] = bag.readAll(topic1);
    fprintf('Got %i messages, first one at time %f\n', length(msgs), meta{1}.time.time);

    %% Read messages incrementally
    bag.resetView(topic1);
    count = 0;
    while bag.hasNext();
        [msg, meta] = bag.read();
        count = count + 1;
    end

    %% See definitions of messages contained within the bag
    twist_definition = bag.definition('geometry_msgs/Twist')

    % Setting 'raw' to true shows the original message definition with comments
    raw = true;
    raw_twist_definition = bag.definition('geometry_msgs/Twist', raw)

    % When it's unambiguous, you can drop the package name.  You can also get
    % definitions for messages defined in other messages;
    % geometry_msgs/Quaternion comes from geometry_msgs/Twist
    quaternion_definition = bag.definition('Quaternion', true)

    %% Convert velocity messages to a matrix to plot linear speed
    [msgs, meta] = bag.readAll(topic1); % Messages are structs
    accessor = @(twist) twist.linear;
    [xyz] = ros.msgs2mat(msgs, accessor); % Convert struct to 3-by-N matrix of linear velcoity
    times = cellfun(@(x) x.time.time, meta); % Get timestamps
    % Plot linear speed over time
    plot(times, xyz(1, :));
    ylim([-0.1 0.1]);
    
elseif choice ==2
    %% Re-read msgs on topic1 and get their metadata
    [msgs, meta] = bag.readAll(topic2);
    fprintf('Got %i messages, first one at time %f\n', length(msgs), meta{1}.time.time);

    %% Read messages incrementally
    bag.resetView(topic2);
    count = 0;
    while bag.hasNext();
        [msg, meta] = bag.read();
        count = count + 1;
    end

    %% See definitions of messages contained within the bag
    twist_definition = bag.definition('geometry_msgs/Pose')

    % Setting 'raw' to true shows the original message definition with comments
    raw = true;
    raw_twist_definition = bag.definition('geometry_msgs/Pose', raw)

    % When it's unambiguous, you can drop the package name.  You can also get
    % definitions for messages defined in other messages;
    % geometry_msgs/Quaternion comes from geometry_msgs/Twist
    quaternion_definition = bag.definition('Quaternion', true)

    %% Convert velocity messages to a matrix to plot linear speed
    [msgs, meta] = bag.readAll(topic2); % Messages are structs
    accessor = @(pose) pose.position;
    [xyz] = ros.msgs2mat(msgs, accessor); % Convert struct to 3-by-N matrix of linear velcoity
    times = cellfun(@(x) x.time.time, meta); % Get timestamps
    % Plot linear speed over time
    plot(times, xyz(1, :));
    ylim([-1.1 1.1]);
else 
    choice=1;
end



%% Learn more
%doc ros.Bag

