function [time_vec,msg] = read_poseStamped(bag_reader)
%READ_POSESTAMPED read poseStamped messages from a bag file
    numMsgs = bag_reader.NumMessages;
    time_vec = zeros(1,numMsgs);
    bag_msgs = readMessages(bag_reader);

    msg = zeros(6,numMsgs);
    for i=1:numMsgs
        t = double(bag_msgs{i}.header.stamp.sec) + double(bag_msgs{i}.header.stamp.nanosec)*10^-9 - bag_reader.StartTime;
        time_vec(i) = t;
        msg(1,i) = bag_msgs{i}.twist.linear.x;
        msg(2,i) = bag_msgs{i}.twist.linear.y;
        msg(3,i) = bag_msgs{i}.twist.linear.z;
        msg(4,i) = bag_msgs{i}.twist.angular.x;
        msg(5,i) = bag_msgs{i}.twist.angular.y;
        msg(6,i) = bag_msgs{i}.twist.angular.z;

    end


end


