%% Waypoint tracking demonstration using Robotics System Toolbox

% This demonstration performs inverse kinematics of a
% robot manipulator to follow a desired set of waypoints.

% Copyright 2017-2018 The MathWorks, Inc.

%% Load and display robot
clear all
close all
clc

addpath(genpath(strcat(pwd,'\Dependencies')))
robot = createRigidBodyTree;
axes = show(robot);
axes.CameraPositionMode = 'auto';

%% Create a set of desired wayPoints
%door
% range = [0  pi*0.45];
dt = 0.1;  %0.2
% w = pi*0.4/4;
% t = range(1):w*dt:range(end);
% r = 0.4;
% c = [0; r];
% % base
% range_b = [0  0.60];
% base_move = range_b(2)-range_b(1);
% x_f = range_b(1):base_move/(length(t)-1):range_b(end);
% x_c = r*sin(t)+0.4 - x_f;
% y_c = r*(1-cos(t));
% z_c = 0.1*ones(1,size(x_c,2));
% wayPoints =[x_c' y_c' z_c'];

load('open_door_ejth.mat')

%wayPoints = [0.35 0 -0.05; 0.27 0 0.1; 0.1 0 0.2];

% wayPoints = [0.2 -0.2 0.02;0.25 0 0.15; 0.2 0.2 0.02]; % Alternate set of wayPoints
%wayPoints = [0.2 -0.2 0.02;0.15 0 0.28;0.15 0.05 0.2; 0.15 0.09 0.15;0.1 0.12 0.1; 0.04 0.1 0.2;0.25 0 0.15; 0.2 0.2 0.02];
%exampleHelperPlotWaypoints(wayPoints);

end_effector = end_effector - traj_ - [ 0 0 0.1]'; 
exampleHelperPlotWaypoints(end_effector');
%% Create a smooth curve from the waypoints to serve as trajectory
exampleHelperPlotWaypoints(end_effector');
%trajectory = cscvn(wayPoints');
trajectory = cscvn(end_effector);
trajectory2 = cscvn(traj_);
% Plot trajectory spline and waypoints
hold on
fnplt(trajectory,'r',2);

%% Perform Inverse Kinematics for a point in space
ik = robotics.InverseKinematics('RigidBodyTree',robot);
weights = [0.1 0.1 0 1 1 1];
initialguess = robot.homeConfiguration;
%%
% Calculate the inverse kinematic solution using the "ik" solver 
% Use desired weights for solution (First three are orientation, last three are translation)
% Since it is a 4-DOF robot with only one revolute joint in Z we do not
% put a weight on Z rotation; otherwise it limits the solution space
numTotalPoints = 30;

% Evaluate trajectory to create a vector of end-effector positions
eePositions = ppval(trajectory,linspace(0,trajectory.breaks(end),numTotalPoints));
bacePositions = ppval(trajectory2,linspace(0,trajectory2.breaks(end),numTotalPoints));

%%
% Call inverse kinematics solver for every end-effector position using the
% previous configuration as initial guess
for idx = 1:size(eePositions,2)
    tform = trvec2tform(eePositions(:,idx)');
    th = 0;%(numTotalPoints-idx)*pi/180;
    tform(1:3,1:3) = [cos(th) 0 sin(th);
                       0      1   0    ;
                     -sin(th) 0 cos(th)];
    configSoln(idx,:) = ik('end_effector',tform,weights,initialguess);
    initialguess = configSoln(idx,:);
end

angles = [configSoln.JointPosition];
thetas = [angles(numTotalPoints+1:numTotalPoints*2);
          angles(numTotalPoints*2+1:numTotalPoints*3);
          angles(numTotalPoints*3+1:numTotalPoints*4)];
          

%% Visualize robot configurations
%fnplt(trajectory,'r',2);
%hold on
title('Robot waypoint tracking visualization')
axis([-0.2 0.6 -0.35 0.35 -0.1 0.35]);
for idx = 1:size(eePositions,2)
    show(robot,configSoln(idx,:), 'PreservePlot', false,'Frames','off');
    pause(0.1)
    hold on
end
hold off

%%


% TB: trajectory dimension
dim         = 2; %x,y
%%%%%%%%%%%%%%%%%%%%%%%%%%%
gr_push_hold
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Arm parameters
robot1=robotproperty_hc(4);
nn = size(thetas,2);
%x_f = -0.20:0.60/(nn-1):0.40;
figure
gap = 1;
theta_1 = [zeros(1,nn); angles(1:numTotalPoints);thetas];
traj_1 = bacePositions+[0.15 0 0]';
%traj_1 = [x_f; zeros(1,nn)];
%plot_MM5(nn,theta_,traj_1(1:2,:),robot1,tb3,gap,1)
[end_effector]=get_end(theta_1,traj_1(1:2,:),robot1);
theta_open = [zeros(1,nn); angles(1:numTotalPoints);thetas];
traj_open = traj_1;

end_effectord = [0 0.35 0 ]' - [-1 0 0 ; 0 1 0; 0 0 -1]*end_effector(1:3,:);
%theta_ = atan([   0.0039 0.0079 0.0109 end_effector(1,4:end)]./end_effector(2,:))/pi*180;
theta_ = atan(end_effectord(1,:)./end_effectord(2,:))/pi*180;
refpath = [];
for i=1:nn
    refpath = [refpath;traj_open(1:2,i)];
end
z0_ = [traj_open(1:2,1);0;0;0];
[X_out, u] = base_ILQR([],refpath,nn,z0_,var);

% ref.theta = theta_open;
% ref.path = refpath;

%%
Ref.z = zeros(19,nn);
Ref.z(1:2,:) = traj_open(1:2,:);
Ref.z(3,:) = X_out(4,:);
Ref.z(5,:) = X_out(4,:);
Ref.z(6,:) = X_out(3,:);
Ref.z(7,:) = X_out(5,:);
Ref.z(12,:) = theta_open(2,:);
Ref.z(14,:) = theta_open(3,:);
Ref.z(16,:) = theta_open(4,:);
Ref.z(18,:) = theta_open(5,:);
%%
save('data/IK_open1.mat','Ref','X_out','theta_open','traj_open','end_effector','theta_','X_all')