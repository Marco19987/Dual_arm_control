function [time_vec,pose_msg] = read_poseStamped(bag_reader)
%READ_POSESTAMPED read poseStamped messages from a bag file
    numMsgs = bag_reader.NumMessages;
    time_vec = zeros(1,numMsgs);
    bag_msgs = readMessages(bag_reader);

    pose_msg = zeros(7,numMsgs);
    for i=1:numMsgs
        t = double(bag_msgs{i}.header.stamp.sec) + double(bag_msgs{i}.header.stamp.nanosec)*10^-9 - bag_reader.StartTime;
        time_vec(i) = t;
        pose_msg(1,i) = bag_msgs{i}.pose.position.x;
        pose_msg(2,i) = bag_msgs{i}.pose.position.y;
        pose_msg(3,i) = bag_msgs{i}.pose.position.z;
        pose_msg(4,i) = bag_msgs{i}.pose.orientation.w;
        pose_msg(5,i) = bag_msgs{i}.pose.orientation.x;
        pose_msg(6,i) = bag_msgs{i}.pose.orientation.y;
        pose_msg(7,i) = bag_msgs{i}.pose.orientation.z;

    end


end


