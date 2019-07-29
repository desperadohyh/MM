%% Generate reference
% clear all 
% close all

%% General parameters
g = 9.81;

%% TB3 model

tb3.base = [0.065 0.065 -0.195 -0.195;
                0.13  -0.13 -0.13  0.13];
            
tb3.wC = [0    0;
         0.14  -0.14  ;
         0.05 0.05];   % center of circle 
R = 0.03 ;    % Radius of circle 
teta=0:0.5:2*pi ;
for i = 1:2
tb3.wx(i,:) = tb3.wC(1,i)+R*cos(teta);
tb3.wy(i,:) = tb3.wC(2,i)*ones(1,length(teta)) ;
tb3.wz(i,:) = tb3.wC(3,i)+R*sin(teta) ;
end

%% New for MM
% xA, yA, xAd, yAd, v,   t1, t1d, tR, tRd, tL,  
% tLd, t2, t2d, t3, t3d,   t4, t4d,t5,t5d 
Ts = dt;
H = 15;
nstep = H+1;
nu = 6; % acceleration dim
%z0_ = [0 0 0 0 0  0 0 0 0 0  0 0 0 -pi/2 0   pi/3 0 pi/4 0 ]';
z0_ = [0 0 0 0 0  0 0 0 0 0  0 -pi/2 0 0 0   0 0 0 0 ]';
Vref = [0.1; 0];
Thref = [ 0 0  -pi 0 pi/4 0   -pi/4 0 pi/10 0]';

r = 0.0215;
ang = Ts*H*norm(Vref)/r;
ang_v = norm(Vref)/r;
zT = [z0_(1)+Ts*H*Vref(1) 0 Vref' norm(Vref)  0 0 z0_(8)+ang ang_v z0_(10)+ang  ang_v Thref(3) 0 0 0   0 0 0 0]';
nstate = size(z0_,1); % QP variable dim

xref_pre =[];
for i = 1:nstate
    xref_pre = [xref_pre; linspace(z0_(i),zT(i),nstep )];
end  

% rearrange reference
xref =[];        
for i=1:nstep
    xref = [xref; xref_pre(:,i)];
end
uref = zeros(H*nu,1);

var.H = H;
var.Vref = Vref;
var.r = r;

%% Init
pathall =[];
trajectory =[];
theta_implement =[];
traj_implement = [];
end_implement = [];
pla_implement = [];
all_implement = [];
fval_all = [];


%% ILQR
var.dt = dt;
NILQR=nstep;
var.N = NILQR;
    
    var.nstep = nstep;
    var.nx = 5;
    var.nu = 2;
    var.Thres=1e-4;
    var.lineSearchThres=1e-4;
    %dim = 2;
    
    
u0 = zeros(2,NILQR-1);
xref_ = [xref_pre(1:2,1:NILQR); zeros(3,NILQR)];


%% Cost

R = blkdiag(eye(2),eye(4));
Q2 = [5 0 ;
      0 1 ];
  
Q3 = [5 0 0 0 0   0 0 0 0 0;
      0 9 0 0 0   0 0 0 0 0;
      0 0 4 0 0   0 0 0 0 0;
      0 0 0 9 0   0 0 0 0 0;
      0 0 0 0 3   0 0 0 0 0;
      
      0 0 0 0 0   8 0 0 0 0;
      0 0 0 0 0   0 2 0 0 0;
      0 0 0 0 0   0 0 7 0 0;
      0 0 0 0 0   0 0 0 1 0;
      0 0 0 0 0   0 0 0 0 6 ];
  
  
Ey = kron(eye(H),[0 1 zeros(nstate - 2,1)']);

Ev = kron(eye(H), [0 0 1 zeros(nstate - 3,1)';
                   0 0 0 1 zeros(nstate - 4,1)' ]);
               
               
Eth = kron(eye(H), [zeros(5,1)' 1 zeros(nstate - 6,1)';
                    zeros(6,1)' 1 zeros(nstate - 7,1)';
                    zeros(8,11) eye(8)]);
                
Aug = diag([ones(1,H-1) 10]);

Raug = kron(Aug, R);
Q2aug = kron(Aug, Q2);
Q3aug = kron(Aug, Q3);

vref = kron(ones(H,1),Vref);
thref = kron(ones(H,1),Thref);

                




