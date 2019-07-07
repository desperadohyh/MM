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
range = [0  pi*0.45];
dt = 0.2;
w = pi*0.4/4;
t = range(1):w*dt:range(end);
r = 0.4;
c = [0; r];
% base
range_b = [0  0.60];
base_move = range_b(2)-range_b(1);
x_f = range_b(1):base_move/(length(t)-1):range_b(end);
x_c = r*sin(t)+0.4 - x_f;
y_c = r*(1-cos(t));
z_c = 0.1*ones(1,size(x_c,2));
wayPoints =[x_c' y_c' z_c'];
%wayPoints = [0.35 0 -0.05; 0.27 0 0.1; 0.1 0 0.2];

% wayPoints = [0.2 -0.2 0.02;0.25 0 0.15; 0.2 0.2 0.02]; % Alternate set of wayPoints
%wayPoints = [0.2 -0.2 0.02;0.15 0 0.28;0.15 0.05 0.2; 0.15 0.09 0.15;0.1 0.12 0.1; 0.04 0.1 0.2;0.25 0 0.15; 0.2 0.2 0.02];
exampleHelperPlotWaypoints(wayPoints);

%% Create a smooth curve from the waypoints to serve as trajectory

trajectory = cscvn(wayPoints');

% Plot trajectory spline and waypoints
hold on
fnplt(trajectory,'r',2);

%% Perform Inverse Kinematics for a point in space
ik = robotics.InverseKinematics('RigidBodyTree',robot);
weights = [0.1 0.1 0 1 1 1];
initialguess = robot.homeConfiguration;

% Calculate the inverse kinematic solution using the "ik" solver 
% Use desired weights for solution (First three are orientation, last three are translation)
% Since it is a 4-DOF robot with only one revolute joint in Z we do not
% put a weight on Z rotation; otherwise it limits the solution space
numTotalPoints = 30;

% Evaluate trajectory to create a vector of end-effector positions
eePositions = ppval(trajectory,linspace(0,trajectory.breaks(end),numTotalPoints));

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
% sampling time
dt          = 0.5;

% TB: trajectory dimension
dim         = 2; %x,y
%%%%%%%%%%%%%%%%%%%%%%%%%%%
gr_push_hold
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Arm parameters
robot1=robotproperty_hc(4);
nn = size(thetas,2);
x_f = -0.20:0.60/(nn-1):0.40;
figure
gap = 5;
plot_implement(nn,[zeros(1,nn); angles(1:numTotalPoints);thetas],[x_f; zeros(1,nn)],robot1,tb3,gap)

theta_open = [zeros(1,nn); angles(1:numTotalPoints);thetas];
traj_open = [x_f; zeros(1,nn)];