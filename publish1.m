%%
%clear all

%%
%load('nice_result_data.mat')
rosshutdown
rosinit('http://10.42.0.137:11311')
sub = rossubscriber('/odom','nav_msgs/Odometry');
subj = rossubscriber('/om_with_tb3/joint_states','sensor_msgs/JointState'); 
subg = rossubscriber('/om_with_tb3/gripper_position','std_msgs/Float64MultiArray'); 

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
thetas = rosmessage('std_msgs/Float64MultiArray');
griper_position = rosmessage('std_msgs/Float64MultiArray');
%speed = rosmessage('std_msgs/Float64MultiArray');

% store value

value = [];

ss = size(theta_out,2);
theta_implement = [theta_out(2:5,:); linspace(0.015,-0.005,ss)];

%% reset
disp('reset');
msgj = receive(subj,10);

current_effort = msgj.Effort(3:7)

current_joints = msgj.Position(3:7);
reset = [linspace(current_joints(1), theta_implement(1,1),10);
        linspace(current_joints(2), theta_implement(2,1),10);
        linspace(current_joints(3), theta_implement(3,1),10);
        linspace(current_joints(4), theta_implement(4,1),10);
        linspace(0.007, theta_implement(5,1),10)];

for nstep_r = 1:10
    tic
    current_effort = msgj.Effort(3:7)
    thetas.Data = [reset(1,nstep_r),reset(1:4,nstep_r)'];
    %speed.Data = [time,acceleration];
    %send(pubM,thetas);
    
    
    griper_position.Data = [reset(5,nstep_r)];
    send(pubG,griper_position);

    
    waitfor(r);
    toc
    
end
r = robotics.Rate(2);


disp('ros');
a=1;

%%
%load('nice_result_data.mat')
%%
% pause(10)

for nstep = 1:ss
    tic
%     if nstep >15
%         a=0.5;
%     end
current_effort = msgj.Effort(3:7)
    cmv.Linear.X = ref.v(nstep)*a;
    cmv.Angular.Z = ref.w(nstep);
    %send(pub,cmv);
    
%     theta1 = ref.theta2(nstep);
%     theta2 = ref.theta3(nstep);
%     theta3 = ref.theta4(nstep);
%     theta4 = ref.theta5(nstep);
    %time = 0.1;
    %acceleration = 0.75;
    
    theta1 = theta_implement(1,nstep);
    theta2 = theta_implement(2,nstep);
    theta3 = theta_implement(3,nstep);
    theta4 = theta_implement(4,nstep);   
    
    thetas.Data = [theta_implement(1,nstep),theta_implement(1:4,nstep)'];
    %send(pubM,thetas);
    griper_position.Data = theta_implement(5,nstep);
    send(pubG,griper_position);
    %speed.Data = [time,acceleration];
    
    
    %%send(pubS,speed);
    
    msgj = receive(subj,10);
    current_joints = msgj.Position(3:6);
    value = [value current_joints];
%     if nstep == 1
%         value = current_joints;
%     else
%         value = horzcat(value,current_joints);
%     end
    disp(nstep)
    %disp(msgj.Position(3:6))
%     joint1 = current_joints(1);
%     joint2 = current_joints(2);
%     joint3 = current_joints(3);
%     joint4 = current_joints(4);
%     
%     plot(nstep,current_joints)
%     hold on
    
    waitfor(r);
    toc
    
end

%%
figure(1)
x = 1:size(theta_implement,2);
plot(x,value(1,:),'-b',x,value(2,:),'-r',x,value(3,:),'-g',x,value(4,:),'-c')
hold on
title('Comparsion of joint angles')
xlabel('Number of nsteps')
ylabel('Angle(Radian?)')
legend('joint1','joint2','joint3','joint4')
plot(x,theta_implement(2,:),'--b',x,theta_implement(3,:),'--r',x,theta_implement(4,:),'--g',x,theta_implement(5,:),'--c')
hold off
%legend('desired1','desired2','desired3','desired4')

% figure(2)
% x = 1:size(ref.x,2);
% plot(x,theta_implement(3,:),x,theta_implement(4,:),x,theta_implement(5,:))
% title('Desired joint angles')
% xlabel('Number of nsteps')
% ylabel('Angle(Radian?)')
% legend('joint1','joint2','joint3')     %2,3,4 --> 1,2,3

cmv.Linear.X = 0;
cmv.Angular.Z = 0;
send(pub,cmv);


%%
rosshutdown
