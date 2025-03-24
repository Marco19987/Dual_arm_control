function [time_vec,msg] = read_wrenchStamped(bag_reader)
%READ_POSESTAMPED read poseStamped messages from a bag file
    numMsgs = bag_reader.NumMessages;
    time_vec = zeros(1,numMsgs);
    bag_msgs = readMessages(bag_reader);

    msg = zeros(6,numMsgs);
    for i=1:numMsgs
        t = double(bag_msgs{i}.header.stamp.sec) + double(bag_msgs{i}.header.stamp.nanosec)*10^-9 - bag_reader.StartTime;
        time_vec(i) = t;
        msg(1,i) = bag_msgs{i}.wrench.force.x;
        msg(2,i) = bag_msgs{i}.wrench.force.y;
        msg(3,i) = bag_msgs{i}.wrench.force.z;
        msg(4,i) = bag_msgs{i}.wrench.torque.x;
        msg(5,i) = bag_msgs{i}.wrench.torque.y;
        msg(6,i) = bag_msgs{i}.wrench.torque.z;

    end


end


