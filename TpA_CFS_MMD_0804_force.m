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
% cost ratio
% y, v, th, u
c = [50 0 1 0.1];
%%%%%%%%%%%%%%%%%%%%%%%%%%%
gen_ref_MMD_0726
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot=robotproperty_MMD(4, z0_, Ts);
% Arm joint
njoint      = 5; % joint number
DH          = robot.DH;
r           = robot.r;
d           = robot.d;
% Arm obs
% center position (1.05,-0.2)
obs_arm     =[[1.05;-0.6;0.35] [1.05;-0.6;0.5]];
obs_arm_r   = 0.35; % radius
ss = 35;






%%
for steps=1:ss

[xori,xref,xref_pre,xR, robot.A, robot.B]=loop_MMD(var,z0_,zT,H,zTt);


%% MM Cost function 
% if steps == 11||steps == 12
%     yd=yd+0.07;
% end
%     obs{1}.poly = [1.1+xd 1.3+xd 1.4+xd 0.9+xd;0.1+yd 0.1+yd -0.5+yd -0.5+yd];
%     
    % Cost Fn Parameters
    
    tic
for k = 1:10

    
    Aaug=[kron(ones(H,1),robot.A)];Baug=kron(tril(ones(H)),robot.B);
    
    for i=1:H-1
        U0_i = uref((i-1)*nu+1:i*nu);    
        Aaug = blkdiag(eye(nstate*i),kron(eye(H-i),robot.A))*Aaug;
        for j=1:i
            Baug(:,(j-1)*nu+1:j*nu)=blkdiag(eye(nstate*i),kron(eye(H-i),robot.A))*Baug(:,(j-1)*nu+1:j*nu);
        end
        [Z1, robot.A, robot.B ] = LinKin(xref(nstate*(i-1)+1:nstate*i), U0_i, var.dt);
    end
    
    
%     % Quadratic term
%     QA =  c(1)*Baug'*Ey'*Ey*Baug + c(2)*Baug'*Ev'*Q2aug*Ev*Baug + c(3)*Baug'*Eth'*Q3aug*Eth*Baug + c(4)*Raug;
%     % Linear term
%     fA = [c(1)*z0_'*Aaug'*Ey'*Ey*Baug + c(2)*(z0_'*Aaug'*Ev'-vref')*Q2aug*Ev*Baug + c(3)*(z0_'*Aaug'*Eth'-thref')*Q3aug*Eth*Baug]';
      
    % Quadratic term
    QA =  c(1)*Baug'*Ep'*Ep*Baug + c(2)*Baug'*Ev'*Q2aug*Ev*Baug + c(3)*Baug'*Eth'*Q3aug*Eth*Baug + c(4)*Raug;
    % Linear term
    fA = [c(1)*(z0_'*Aaug'*Ep'-pref')*Ep*Baug + c(2)*(z0_'*Aaug'*Ev'-vref')*Q2aug*Ev*Baug + c(3)*(z0_'*Aaug'*Eth'-thref')*Q3aug*Eth*Baug]';
  

        


% %% The constraint
%     D=obs_arm_r;
%     LA=[];SA=[];
%     LLA=[];
%     SSA=[];
%     I=[];
%     rec_d = [];
%     for i=1:H
%         % provide base according to current 2D path 
%         xy = xref(i*nstate+1:i*nstate+2);
%         base = [xy' 0.1];
%         % get reference theta
%         theta=[xref_pre(6,i+1) xref_pre(12:2:18,i+1)']';
%         [distance,linkid]=dist_arm_3D_Heu_hc(theta,DH(1:njoint,:),base,obs_arm,robot.cap);
%         rec_d = [rec_d distance];
%         
%         I = [I;distance-D];
%         ff = @(x) dist_arm_3D_Heu_hc(x,DH(1:njoint,:),base,obs_arm,robot.cap);
%         Diff = num_jac(ff,theta); Diff = Diff';
%         
%         Bj=Baug((i-1)*nstate+1:i*nstate,1:H*nu);
%         s=I(i)-Diff'*Bj(1:njoint,:)*uref;
%         l=-Diff'*Bj(1:njoint,:);
%         %ll = reshape(l,[5,H]);
%         lL = [];
%         for i_re = 1:H
%             lL = [lL l((i_re-1)*5+1)*(r/d) -l((i_re-1)*5+1)*(r/d) l((i_re-1)*5+2:i_re*5)];
%         end
%         LLA = [LLA;lL];
%         SSA = [SSA;s];
%         
%         
% %         LA=[LA;  lL  ];
% %         SA=[SA;s];
%         
%          % Soft constraint
% %         LA=[LA; [ lL  zeros(1,i-1) -1 zeros(1,H-i)]];
% %         SA=[SA;s];
% %         LA = [LA;[ zeros(1,H*6)  zeros(1,i-1) -1 zeros(1,H-i)]];
% %         SA = [SA;0];
%     end
    %% acceleration limit
    LA = eye(H*nu);
    acc_lim = 2*[pi/2 pi/2 pi/10 pi/10 pi/5 pi/5]';
    SA = kron(ones(H,1),acc_lim);
    
    
    %% EE force constrain
    LA = [LA zeros(size(LA,2),H)];
    for i=1:H
        zk = xref(nstate*(i-1)+1:nstate*i);
        u = uref((i-1)*nu+1:i*nu);
        
    Mk_Z = Mk_f(zk(9),zk(11),zk(12),zk(13),zk(14),zk(15),zk(16),zk(17),zk(18),zk(19),zk(6),zk(9),zk(11));
    Vk_Z = Vk_f(zk(9),zk(11),zk(12),zk(13),zk(14),zk(15),zk(16),zk(17),zk(18),zk(19),zk(6),zk(9),zk(11),zk(13),zk(15),zk(17),zk(19));
    Gk_Z = Gk_f(zk(14),zk(16),zk(18));
    Jk_Z = Jk_f(zk(9),zk(11),zk(12),zk(13),zk(14),zk(15),zk(16),zk(17),zk(18),zk(6),zk(9),zk(11));
    Jdk_Z = Jdk_f(u(1),u(2),u(3),u(4),u(5),zk(9),zk(11),zk(12),zk(13),zk(14),zk(15),zk(16),zk(17),zk(18),zk(19),zk(6),zk(9),zk(11)); 
    
    Jrv_Z = Jrv_f(zk(6),zk(12),zk(14),zk(16),zk(18));
    dJrv_Z = dJrv_f(zk(6),zk(7),zk(12),zk(13),zk(14),zk(15),zk(16),zk(17),zk(18),zk(19));
    Jr_z = Jr_f(zk(6),zk(12),zk(14),zk(16),zk(18));
    
%     Jit = pinv(Jk_Z)';
%     Ji = pinv(Jk_Z);
%     Mx = Jit*Mk_Z*Ji;
%     Vx = Jit*(Vk_Z-Mk_Z*Ji*Jdk_Z*zk(9:2:end));
%     Gx = Jit*Gk_Z;
%     
%     Xdd = Jdk_Z*zk(9:2:end)+Jk_Z*u;
%     b_force = Mx*(Jdk_Z*zk(9:2:end))+Vx+Gx;
%     al =  Mx*Jk_Z;
    
    Jit = pinv(Jr_z)';
    Mx = Jit*Mk_Z;
    Vx = Jit*Vk_Z;
    Gx = Jit*Gk_Z;
    force = Mx*u+Vx+Gx;
    b_force = Vx+Gx;
    al =  Mx;
    
    
    LA = [LA;  zeros(1,(i-1)*nu) al(1,:) zeros(1,(H-i)*nu) zeros(1,i-1) -1 zeros(1,H-i)];
    SA = [SA;  F-b_force(1) ];
    LA = [LA;  zeros(1,(i-1)*nu) -al(1,:) zeros(1,(H-i)*nu) zeros(1,i-1) -1 zeros(1,H-i)];
    SA = [SA;  -F+b_force(1) ];
    LA = [LA;  zeros(1,H*nu)  zeros(1,i-1) -1 zeros(1,H-i)];
    SA = [SA;  0 ];
    
    end


%% QP
%     Q = QA;
%     f = fA;

   % Quadratic term
    soft = 0.1;
    Q = blkdiag(QA,soft*eye(H));
    
    % Linear term
    f = [fA'  zeros(H,1)']';
    
    options =  optimoptions('quadprog','Display','off');
    [soln,fval] = quadprog(Q,f,LA,SA,[],[],[],[],[],options);
%    soln = quadprog(QA,fA,LA,SA,[],[],[],[],[],options);
    % TB path update 
    alpha_out = soln(1:6*H);
    states_out = Aaug*z0_+Baug*alpha_out;
    states_out = [z0_ ;states_out];
    theta_out = [  z0_(6)  states_out(6:nstate:end)';
                   z0_(12)  states_out(12:nstate:end)';
                   z0_(14)  states_out(14:nstate:end)';
                   z0_(16)  states_out(16:nstate:end)';
                   z0_(18)  states_out(18:nstate:end)'];
               
    waypoints = [  states_out(1:nstate:end)';
                   states_out(2:nstate:end)'];
    
    
    uref=[alpha_out(7:end); zeros(6,1)] ;
    
    
    
    % break criteria 
    if norm(xref-states_out) < 0.00001 
        pathall = [pathall waypoints]; 
        disp(['converged at step ',num2str(k)]);
        break
    end
%     if norm(xori-states_out) < 0.1 && all(LLA*uref<SSA)
%         pathall = [pathall waypoints];
%         disp(strcat('Local Optimal Found at step ',num2str(k)));
%         %break;
%     end
    xref = states_out;
    
    

    
end
Xdd = Jdk_Z*z0_(9:2:end)+Jk_Z*alpha_out(1:6);
force = Mx*Xdd+Vx+Gx;
force_implement = [force_implement double(force)];
k
toc
%%   
    traj=zeros(5,NILQR);
    traj(1,:) = states_out(1:nstate:end)';
    traj(2,:) = states_out(2:nstate:end)';
    traj(3,:) = states_out(6:nstate:end)';
    traj(4,:) = states_out(5:nstate:end)';
    traj(5,:) = states_out(7:nstate:end)';
    ILQRtraj = traj(:,1:NILQR);
    xref_= ILQRtraj;
    
    
    L1 = zeros(7,NILQR);
    
    L2 = diag([1000 1000 0 0 0 10 10 ]);
    L2=reshape(repmat(L2,1,NILQR),7,7,NILQR);
    l=zeros(1,NILQR);
    
    u0 = zeros(2,NILQR-1);
    Tx_current = [z0_(1:2); z0_(6); z0_(5); z0_(7)];

    %xref_
    [ X_out, u, xbar, ubar, kk, K, sigsu, A, B ] = MaxEntILQR( L2, L1, l, Tx_current, u0, var, xref_ );

    %% Plot

end_effector = [];
T = [0   0 -0.15    0   0.03 0.13;
     0   0   0      0  -0.12   0 ;
     0   0   0.0    0     0    0];
for j = 1:nstep
     
    %for i=1:nstate
    theta=theta_out(:,j);
    DHn = [zeros(1,4);DH];
    
    nlink=size(DH,1);
    pos=cell(1,nlink);
    M=cell(1,nlink+1); M{1}=eye(4);

    for i=2:nlink+1
        % R 
        R=[cos(theta(i-1)) -sin(theta(i-1))  0;
            sin(theta(i-1)) cos(theta(i-1))  0;
              0                        0                 1];
               
        if i == 4
            Rx=[1     0             0        ;  
                0  cos(-0.5*pi) -sin(-0.5*pi); 
                0  sin(-0.5*pi) cos(-0.5*pi) ];
                   
            R = Rx*R;
        end
        M{i}=M{i-1}*[R T(:,i); zeros(1,3) 1]; 
        
end
    
    end_effector(:,j)=M{i}(1:3,1:3)*[0.08;0;0]+M{i}(1:3,4)+[waypoints(1,j);waypoints(2,j);0.1];

end
%%


figure(fighandle(1));
%%%%%%%%%%%%%%%%%%%%%%
z0_ = states_out(nstate+1:nstate*2);
Tx_current =  X_out(6:10)';
Ax_current =  theta_out(:,2);

g_current = 0; %%%%%%%%%%%%%

theta_implement = [theta_implement [Ax_current; g_current]];
end_implement = [end_implement  end_effector(:,2)];
traj_implement = [traj_implement X_out(6:7)'];
pla_implement = [pla_implement X_out(13:15)'];
all_implement = [all_implement z0_];
fval_all = [fval_all fval];


    
%if steps == 3||steps == 8||steps == 13 ||steps == 18||steps == 23||steps == 28||steps == 33||steps == 38
end_ = plot3(end_effector(1,:),end_effector(2,:),end_effector(3,:),'-*','color',[1-(steps/ss),1-(steps/ss),1-(steps/ss)]);

hold on
traLQ_= plot(X_out(1,:),X_out(2,:),'-*','color',[1-(steps/ss),1-(steps/ss),1-(steps/ss)]);

hold on
%end
%pause
end
%%
figure
gap = 1;
plot_MM5(ss,theta_implement(1:end-1,:),traj_implement,robot,tb3,gap,1)
%plot_implement(ss,theta_implement(1:end-1,:),traj_implement,robot,tb3,gap,0)

%%
figure(fighandle(1));
r=0.14;
[X,Y,Z] = cylinder(r);
h=mesh(X+1.18,Y-.1,Z*0.5,'facecolor',[1 0 0]);
etrj = plot3(end_implement(1,:),end_implement(2,:),end_implement(3,:),'*r-');
ptrj = plot(traj_implement(1,:),traj_implement(2,:),'-or');
hold on
axis equal
axis([-0.5 3.5 -.6 .5 -0.5 1.2]);
%legend([etrj ptrj Obs h],'end-effector','platform','obs. (for platform)','obs. (for arm)','location','eastoutside')
view(3);
%subplot(1,2,2);
xlabel('x[m]')
ylabel('y[m]')
zlabel('z[m]')
%%
PPL = 0;
if PPL == 1
%%
for j = 1:nstep-1
    v_out(j) = norm([x_out(j+1)-x_out(j),y_out(j+1)-y_out(j)]);
end

ref.x = X_out(1,:);
ref.y = X_out(2,:);
ref.v = X_out(4,:);
ref.w = X_out(5,:);


ref.theta1 = theta_out(1,:);
ref.theta2 = theta_out(2,:);
ref.theta3 = theta_out(3,:);
ref.theta4 = theta_out(4,:);
ref.theta5 = theta_out(5,:);

%%
figure
yyaxis left
plot(ref.v,'-bx')
%ylabel('Linear velocity [m/s]')
hold on
yyaxis right
plot(ref.w,'-ro')
%ylabel('Angular velocity [rad/s]')
legend('Vel','AngVel','location','best')
%xlabel('Time step')

%%
figure
plot(ref.theta2,'m-d')
hold on
plot(ref.theta3,'--x')
plot(ref.theta4,'-r*')
plot(ref.theta5,'-go')
legend('\theta_1','\theta_2','\theta_3','\theta_4','location','eastoutside')
ylabel('Angle[rad]')
xlabel('Time step')
end

%%
figure
plot(theta_implement(1,:),'-ko')
hold on
plot(theta_implement(2,:),'m-d')
hold on
plot(theta_implement(3,:),'--x')
plot(theta_implement(4,:),'-r*')
plot(theta_implement(5,:),'-go')
legend('platform angle','\theta_1','\theta_2','\theta_3','\theta_4','location','eastoutside')
ylabel('Angle[rad]')
xlabel('Time step')

ref.v = pla_implement(2,:);
ref.w = pla_implement(3,:);

figure
yyaxis left
plot(ref.v,'-bx')
ylabel('Linear velocity [m/s]')
hold on
yyaxis right
plot(ref.w,'-ro')
ylabel('Angular velocity [rad/s]')
legend('Vel','AngVel','location','best')
xlabel('Time step')

%% force
figure

plot(force_implement(1,:))
hold on
plot(force_implement(2,:))
plot(force_implement(3,:))


legend('X_{e,w}', 'Y_{e,w}','Z_{e,w}')
