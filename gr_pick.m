%% Generate reference
% clear all 
% close all

%% Arm

% initial point and goal point
z0 = [0;-pi/2;0;0;0];
Ax_current = [0;-pi/2;0;0;0];
%zT = [0;-pi/2;0;0;0];
zAT = [0;-pi;pi/6;-pi/6;pi/6];

% zT = [-pi/2;-pi/2;0;0;0];
% zAT = [-pi/2;-pi/2;0;0;0];
% horizon settings
horizon = 24;
nstep = horizon + 1;

% Generate reference
xref_pre = [linspace(z0(1),zAT(1),nstep);
            linspace(z0(2),zAT(2),nstep);
            linspace(z0(3),zAT(3),nstep);
            linspace(z0(4),zAT(4),nstep);
            linspace(z0(5),zAT(5),nstep);
            zeros(5,nstep)];
        

% rearrange reference
xref =[];        
for i=1:nstep
    xref = [xref; xref_pre(:,i)];
end
uref = zeros(horizon*5,1);

% back up



%% Turtlebot

% initial point and goal point
% x0 = 1;
% y0 = 0.5;
% z0 = [x0;y0];
% zT = [1.3;0.1];

x0 = 2.5;
y0 = 0;
z0 = [x0;y0];
Tx_current = [[x0;y0];0;0;0];
zT = [2;0];
% horizon settings
vision = 2;

% Generate reference
path = [linspace(z0(1),zT(1),nstep);
        linspace(z0(2),zT(2),nstep)];
pathall =[];
trajectory =[];
theta_implement =[];
traj_implement = [];
end_implement = [];
pla_implement = [];
% rearrange reference    
refpath = [];
for i=1:nstep
    refpath = [refpath;path(:,i)];
end

%% ILQR
var.dt = dt;
NILQR=nstep;
var.N = NILQR;
    
    var.nstep = 25;
    var.nx = 5;
    var.nu = 2;
    var.Thres=1e-4;
    var.lineSearchThres=1e-4;
    dim = 2;
    var.nx=5;
    var.nu=2;
    
u0 = zeros(2,NILQR-1);
xref_ = [path; zeros(3,NILQR)];