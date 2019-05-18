%% Generate reference
% clear all 
% close all

%% Arm

% initial point and goal point
z0 = [0;-pi/2;0;0;0];
Ax_current = [0;-pi/2;0;0;0];
zT = [0;-pi/2;0;0;0];
zAT = [0;-pi/2;0;0;0];
% zT = [-pi/2;-pi/2;0;0;0];
% zAT = [-pi/2;-pi/2;0;0;0];
% horizon settings
horizon = 24;
nstep = horizon + 1;

% Generate reference
xref_pre = [linspace(z0(1),zT(1),nstep);
            linspace(z0(2),zT(2),nstep);
            linspace(z0(3),zT(3),nstep);
            linspace(z0(4),zT(4),nstep);
            linspace(z0(5),zT(5),nstep);
            zeros(5,nstep)];
        

% rearrange reference
xref =[];        
for i=1:nstep
    xref = [xref; xref_pre(:,i)];
end
uref = zeros(horizon*5,1);

%% Turtlebot

% initial point and goal point
% x0 = 1;
% y0 = 0.5;
% z0 = [x0;y0];
% zT = [1.3;0.1];

x0 = 0;
y0 = 0;
z0 = [x0;y0];
Tx_current = [[x0;y0];0;0;0];
zT = [2;0];
% horizon settings
horizon = 24;
nstep = horizon+1;
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


%% New for MM
% xA, yA, xAd, yAd, v,   t1, t1d, tR, tRd, tL,  
% tLd, t2, t2d, t3, t3d,   t4, t4d,t5,t5d 
Ts = 0.5;
H = 15;
nstep = H+1;

z0_ = [0 0 0 0 0  0 0 0 0 0  0 0 0 -pi/2 0   pi/3 0 pi/4 0 ]';
Vref = [0.2; 0];
ang = Ts*H*norm(Vref)*2*pi/0.215;
ang_v = norm(Vref)*2*pi/0.215;
zT = [z0(1)+1 0 Vref' norm(Vref)  0 0 z0_(8)+ang ang_v z0_(10)+ang  ang_v -pi/2 0 0 0   0 0 0 0];

xref_pre = [linspace(z0_,zT,nstep )];
        

% rearrange reference
xref =[];        
for i=1:nstep
    xref = [xref; xref_pre(:,i)];
end
uref = zeros(horizon*5,1);