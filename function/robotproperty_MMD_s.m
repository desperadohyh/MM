function robot=robotproperty_MMD_s(id, z0_, Ts)

         
        
        %the constants
        nlink = 2;
        robot.nlink = nlink;
        robot.umax=10;
        robot.thetamax=[-pi/2,pi/2; -pi/2,(pi/2)*0.75];
        robot.thetadotmax=[1;1];
        robot.margin=0.05;
        robot.delta_t=0.5;
        robot.tua_max = 3;
        
        %The length of each links and DH parameter and base
        robot.l=[0.12228; 0.1245];
        robot.lc=[0.0618; 0.0645];
        theta = [ 0 ; 0  ];
        robot.theta = theta;
        T = [0   0   0.03 ;
             0   0  -0.12 ;
             0   0    0  ]; %% distance between coordinates
         
        robot.Tc = [   0.015  0.07 ;
                        -0.07   0    ;
                         0      0    ]; % center of mass of each link w.r.t. local coordinates
        robot.DH=[0, 0.04,  0 , 3*pi/2;                    
                  0, 0.12, 0.0235, 0];%theta,d,a,alpha
%         robot.DH=[
%                   0, 0.034, 0.0825, 0 ;
%                   0, 0.04,  0 , 3*pi/2;                    
%                   0, 0.12, 0.0235, 0;
%                   0, 0, 0.1245, 0];%theta,d,a,alpha
        robot.m = [ 0.13 ; 0.12 ]; % mass of each link
        
        Ixyz = [   163    163
                   3.5    3.5
                   163    163]*10^-05; %%%%%%
        
        robot.Ic = cell(1,nlink);
        for i = 1:nlink
            robot.Ic{i} = diag(Ixyz(:,i));
        end
               
        
               
        robot.base=[0;0;0];%origin
        
        robot.cap={};       
        
        robot.cap{1}.p=[0.011 0.011;0 -0.12;0 0];
        robot.cap{1}.r=0.025;
        
        robot.cap{2}.p=[0 0.124;0 0;0 0];
        robot.cap{2}.r=0.025;        
        
        
        robot.base = [z0_(1) z0_(2) 0.1]';
        robot.r = 0.03;
        robot.d = 0.15;
        
        
        
        

%% The kinematic model

U0 = zeros(6,1);

robot.A=[eye(robot.nlink) robot.delta_t*eye(robot.nlink);
        zeros(robot.nlink) eye(robot.nlink)];
robot.B=[0.5*robot.delta_t^2*eye(robot.nlink);
        robot.delta_t*eye(robot.nlink)];
% robot.A = A;
% robot.B = B;
robot.Ac=[eye(3) robot.delta_t*eye(3);
        zeros(3) eye(3)];
robot.Bc=[0.5*robot.delta_t^2*eye(3);
        robot.delta_t*eye(3)];
robot.C=[];
robot.D=[];
robot.Q=[];
robot.R=[];

robot.x(1:robot.nlink*2,1)=[robot.DH(:,1);zeros(robot.nlink,1)];%(theta1,theta2,theta,3,theta1dot,theta2dot,theta3dot)

% Ulternative for CapPos
base = robot.base;
RoCap = robot.cap;
nlink=robot.nlink;
pos=cell(1,nlink);
M=cell(1,nlink+1); M{1}=eye(4);
R_=cell(1,nlink); R_{1}=eye(3);
for i=2:nlink+1
        % R in book
        R=[cos(theta(i-1)) -sin(theta(i-1))  0;
            sin(theta(i-1)) cos(theta(i-1))  0;
              0                        0                 1];
        % T in book
        %T=[DHn(i-1,3);-sin(DHn(i-1,4))*DHn(i,2);cos(DHn(i-1,4))*DHn(i,2)];
       
        if i == 2
            Rx=[1     0             0        ;  
                0  cos(-0.5*pi) -sin(-0.5*pi); 
                0  sin(-0.5*pi) cos(-0.5*pi) ];
                   
            R = Rx*R;
        end
        R_{i-1} = R;
        M{i}=M{i-1}*[R T(:,i); zeros(1,3) 1]; 
        for k=1:2
         pos{i-1}.p(:,k)=M{i}(1:3,1:3)*RoCap{i-1}.p(:,k)+M{i}(1:3,4)+base;
        end
end

robot.R_ = R_;
robot.M = M;
robot.pos = pos;
robot.T = T;

%robot.pos=CapPos(robot.base,robot.DH,robot.cap);

%For SSA
robot.const.P1=[eye(robot.nlink) zeros(robot.nlink);zeros(robot.nlink) zeros(robot.nlink)];
robot.const.P2=[zeros(robot.nlink) 0.5*eye(robot.nlink);0.5*eye(robot.nlink) zeros(robot.nlink)];
robot.const.P3=[zeros(robot.nlink) zeros(robot.nlink);zeros(robot.nlink) eye(robot.nlink)];

%For collision check
robot.profile={};
end


