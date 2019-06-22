function odom_path_msg = record_im(refpath)

poseArray = [];
odom_path_msg = rosmessage('nav_msgs/Path');
odom_path_msg.Header.FrameId = 'odom';
for i = 1:size(refpath,2)
    poses = rosmessage('geometry_msgs/PoseStamped');
    poses.Pose.Position.X = refpath(1,i);
    poses.Pose.Position.Y = refpath(2,i);
    poseArray = [poseArray, poses];
end
odom_path_msg.Poses = poseArray;
end
