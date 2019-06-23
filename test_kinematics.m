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
gr_push_hold
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

ss=15;
xV = -0.15;
Vx = xV;

%% Plot


[xref_t,xref,xR,refpath]=generate_reference_loop_hold(var,Ax_current,Tx_current,zAT,zT,horizon,nstate,u0,mode_,target,t_marg );




pos={};

color{1} = '*r-';
color{2} = '-ko';
color{3} = 'm-d';
color{4} = '--x';
color{5} = '-yo';

for i=1:horizon
    % provide mode_ according to current 2D path 
    xy = refpath(i*2+1:(i+1)*2);
    base = [xy' 0.1];
    % get reference theta
    theta=xref(nstate*(i-1)+1:nstate*(i-1)+njoint);
    [end_dis,pos]=plot_link(theta,DH(1:njoint,:),base,obs_arm,robot.cap);
    for j = 1:5
       etrj = plot3( pos{j}.p(1,3), pos{j}.p(2,3), pos{j}.p(3,3),color{j})
       hold on
    end
    pause
end



% plot





