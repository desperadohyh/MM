%%
%clear all

%%
%load('nice_result_data.mat')
rosshutdown
rosinit('http://10.42.0.1:11311')
sub = rossubscriber('/odom','nav_msgs/Odometry');
subj = rossubscriber('/om_with_tb3/joint_states','sensor_msgs/JointState'); 
pub = rospublisher('/om_with_tb3/cmd_vel','geometry_msgs/Twist');
pubM = rospublisher('/om_with_tb3/joint_trajectory_point','std_msgs/Float64MultiArray');
pubG = rospublisher('/om_with_tb3/gripper_position','std_msgs/Float64MultiArray');
%pubS = rospublisher('/om_with_tb3/joint_speed','std_msgs/Float64MultiArray');
r = robotics.Rate(2);
    
cmv = rosmessage('geometry_msgs/Twist');
theta1 = rosmessage('std_msgs/Float64');
theta2 = rosmessage('std_msgs/Float64');
theta3 = rosmessage('std_msgs/Float64');
theta4 = rosmessage('std_msgs/Float64');

joint_pub1 = rospublisher('/om_with_tb3/joint1_position/command','std_msgs/Float64');
joint_pub2 = rospublisher('/om_with_tb3/joint2_position/command','std_msgs/Float64');
joint_pub3 = rospublisher('/om_with_tb3/joint3_position/command','std_msgs/Float64');
joint_pub4 = rospublisher('/om_with_tb3/joint4_position/command','std_msgs/Float64');

joint_msg1 = rosmessage('std_msgs/Float64');
joint_msg2 = rosmessage('std_msgs/Float64');
joint_msg3 = rosmessage('std_msgs/Float64');
joint_msg4 = rosmessage('std_msgs/Float64');

% joint_msg4 = rosmessage('std_msgs/Float64');%speed = rosmessage('std_msgs/Float64MultiArray');

   

disp('ros');
a=1;

%%
%load('nice_result_data.mat')
%%
pause(10)

for step = 1:size(ref.x,2)
    tic
    if step >15
        a=0.5;
    end
    cmv.Linear.X = ref.v(step)*a;
    cmv.Angular.Z = ref.w(step);
%     send(pub,cmv);
    
%     theta1 = ref.theta2(step);
%     theta2 = ref.theta3(step);
%     theta3 = ref.theta4(step);
%     theta4 = ref.theta5(step);
    %time = 0.1;
    %acceleration = 0.75;
    
    theta1 = theta_out(2,:);
    theta2 = theta_out(3,:);
    theta3 = theta_out(4,:);
    theta4 = theta_out(5,:);
    
    theta1 = theta1(step);
    theta2 = theta2(step);
    theta3 = theta3(step);
    theta4 = theta4(step);
    
    thetas.Data = [theta1,theta1,theta2,theta3,theta4];
    %speed.Data = [time,acceleration];
%     send(pubM,thetas);

    joint_msg1.Data = theta1;
    joint_msg2.Data = theta2;
    joint_msg3.Data = theta3;
    joint_msg4.Data = theta4;

    send(joint_pub1, joint_msg1);
    send(joint_pub2, joint_msg2);
    send(joint_pub3, joint_msg3);
    send(joint_pub4, joint_msg4);
    
    %send(pubS,speed);
    
    msgj = receive(subj,10);
    current_joints = msgj.Position(3:6);
    if step == 1
        value = current_joints;
    else
        value = horzcat(value,current_joints);
    end
    disp(step)
    disp(msgj.Position(3:6))
%     joint1 = current_joints(1);
%     joint2 = current_joints(2);
%     joint3 = current_joints(3);
%     joint4 = current_joints(4);
%     
%     plot(step,current_joints)
%     hold on
    
    waitfor(r);
    toc
    
end

%%
figure(1)
x = 1:size(theta_out,2);
plot(x,value(1,:),'-b',x,value(2,:),'-r',x,value(3,:),'-g',x,value(4,:),'-c')
hold on
title('Comparsion of joint angles')
xlabel('Number of steps')
ylabel('Angle(Radian?)')
legend('joint1','joint2','joint3','joint4')
plot(x,theta_out(2,:),'--b',x,theta_out(3,:),'--r',x,theta_out(4,:),'--g',x,theta_out(5,:),'--c')
hold off
%legend('desired1','desired2','desired3','desired4')

% figure(2)
% x = 1:size(ref.x,2);
% plot(x,theta_out(3,:),x,theta_out(4,:),x,theta_out(5,:))
% title('Desired joint angles')
% xlabel('Number of steps')
% ylabel('Angle(Radian?)')
% legend('joint1','joint2','joint3')     %2,3,4 --> 1,2,3

cmv.Linear.X = 0;
cmv.Angular.Z = 0;
send(pub,cmv);


%%
rosshutdown
