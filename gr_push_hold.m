%% Generate reference
% clear all 
% close all

%% Arm

% initial point and goal point
%z0 = [0;-pi/2;0;0;0];
z0 = [0 ;-pi; pi*0.4386; -pi*0.2498; -pi*0.0277];
Ax_current = z0;
%zT = [0;-pi/2;0;0;0];
%zAT = [0 ;-pi; pi/2; -pi*0.2314; -pi*0.2655];
%zAT = [0 ;-pi; pi*0.4386*2/3; -pi*0.2498; -pi*0.0277]; % theta_{1}
zAT = [0; -pi; 0.8471; -0.2132; 0.8846];
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
zT = [2.5-0.12;0];
% horizon settings
vision = 2;

% Generate reference
path = [linspace(z0(1),zT(1),nstep);
        linspace(z0(2),zT(2),nstep)];
pathall =[];
trajectory =[];
theta_implement =[Ax_current; 0.015];
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

%% load grasping profile
grip_open = 0.015;
grip_close = -0.005;
g_size =10;
g_current = grip_open;

% low weight(long stretch) to large weight(short stretch)
ccc = [load('theta_')];
theta_ = ccc.theta_;
theta_{6} = theta_{5};
move_marg = [  0 0.31 0.27 0.23 0.2 0.2;
               0   0   0   0   0   0;
               0   0   0   0   0   0];

th_size = size(theta_{2},2);
effort_p = [ 400 500 600 700 900 900]; %%%%%%%%%%

%% OBS
obs_c = [1.1;-0.9];
width = [0.4 0.8]/2;
hight = 0.6/2;
% Arm obs
%obs_arm     =[[obs_c;-0.1 ] [obs_c;0.5]];
obs_arm     =[[1.9627;-0.0006;-0.0111 ] [1.9763;-0.0003;-0.0025]];
obs_arm_r   = 0.2; % radius
% TB: obstacle
nobj        = 1;
obs         = {};
obs{1}.v    = [0;0];
margin = 0.2;

xd = 0;
yd = 0;
obs{1}.poly = [ obs_c(1)-width(1)+xd obs_c(1)+width(1)+xd obs_c(1)+width(2)+xd obs_c(1)-width(2)+xd;
                obs_c(2)+hight+yd obs_c(2)+hight+yd obs_c(2)-hight+yd obs_c(2)-hight+yd]; % 4 1 2 3
            
%% Target
target = [-0.2; 0; 0.05];
t_marg = [0.35 0.2 0.35 0; 0  0  0  0; 0 0 0 0];

ss=85;
xV = -0.15;

%% Initializa conditions
pass = 0;
mode_ = 1;
MODE = [];
effort = 5;
idx = 1;
Dt = [];
ee_I =[];
h_D = 0.1;