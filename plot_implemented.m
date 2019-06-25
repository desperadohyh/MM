
%% plot distance

clc
clear all
close all
fighandle = [];
fighandle(1) = figure(1); hold on;
set(gcf, 'position', [0 0 500 500]);
fighandle(2) = figure(2); hold on;

    
%% parameter definition
% TB definition
% sampling time
dt          = 0.5;

% TB: trajectory dimension
dim         = 2; %x,y
%%%%%%%%%%%%%%%%%%%%%%%%%%%
gr_push_hold_new
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Arm parameters
robot=robotproperty_hc(4);
% Arm joint
njoint      =5; % joint number
nstate      =10; % QP variable dim
nu          =5; % acceleration dim 
DH          =robot.DH;





%%
% Target
target = [-0.2; 0; 0.05];
t_marg = [0.35 0.2 0.35 0; 0  0  0  0; 0 0 0 0];

ss=25;
xV = -0.15;
Vx = xV;




[xref_t,xref,xR,refpath]=generate_reference_loop_hold(var,Ax_current,Tx_current,zAT,zT,ss,nstate,u0,mode_,target,t_marg );


theta_implement = [  xref(1:10:end)';
                     xref(2:10:end)';
                     xref(3:10:end)';
                     xref(4:10:end)';
                     xref(5:10:end)'];
                 
                 
traj_implement = [refpath(1:2:end)';refpath(2:2:end)'];
gap = 3;
plot_implement(ss,theta_implement,traj_implement,robot,tb3,gap)






