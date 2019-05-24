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
% TB: obstacle
%nobj        = 1;
% obs         = {};
% obs{1}.v    = [0;0];
% 
% xd = 0;
% yd = 0;
% obs{1}.poly = [1.1+xd 1.3+xd+0.5 1.4+xd+0.5 0.9+xd;0.1+yd 0.1+yd -0.5+yd -0.5+yd];
% TB: trajectory dimension
%dim         = 2; %x,y

% cost ratio
% y, v, th, u
c = [1 1 1 1];
%%%%%%%%%%%%%%%%%%%%%%%%%%%
gen_ref_MMD
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Arm definition
% Arm parameters
robot=robotproperty_MMD(4, z0_, Ts);
% Arm joint
njoint      =5; % joint number

DH          =robot.DH;
r = robot.r;
d = robot.d;

% Arm obs
% center position (1.05,-0.2)
obs_arm     =[[1.05;-0.2;0.35] [1.05;-0.2;0.5]];
obs_arm_r   = 0.35; % radius

ss=50;
%X_out(1) = 0;
xV = 0.2;

%%
for steps=1:ss

[xref_t,xori,xref,xref_pre,xR, robot.A, robot.B]=gen_loop_MMD(var,z0_,zT,H);

%% MM Cost function 
% if steps == 11||steps == 12
%     yd=yd+0.07;
% end
%     obs{1}.poly = [1.1+xd 1.3+xd 1.4+xd 0.9+xd;0.1+yd 0.1+yd -0.5+yd -0.5+yd];
%     
    % Cost Fn Parameters
    Aaug=[kron(ones(H,1),robot.A)];Baug=kron(tril(ones(H)),robot.B);
    U0 = zeros(6,1);
    for i=1:H-1
              
        Aaug = blkdiag(eye(nstate*i),kron(eye(H-i),robot.A))*Aaug;
%         Qaug((i-1)*nstate+1:i*nstate,(i-1)*nstate+1:i*nstate)=Q_arm*0.1;
%         if i==H
%             Qaug((i-1)*nstate+1:i*nstate,(i-1)*nstate+1:i*nstate)=Q_arm*10000;
%         end
        for j=1:i
            Baug(:,(j-1)*nu+1:j*nu)=blkdiag(eye(nstate*i),kron(eye(H-i),robot.A))*Baug(:,(j-1)*nu+1:j*nu);
        end
        [Z1, robot.A, robot.B ] = LinKin(xref(nstate*(i-1)+1:nstate*i), U0, var.dt);
    end
    
    
    % Quadratic term
    QA =  c(1)*Baug'*Ey'*Ey*Baug + c(2)*Baug'*Ev'*Q2aug*Ev*Baug + c(3)*Baug'*Eth'*Q3aug*Eth*Baug + c(4)*Raug;
    % Linear term
    fA = 2*[c(1)*z0_'*Aaug'*Ey'*Ey*Baug + c(2)*(z0_'*Aaug'*Ev'-vref')*Q2aug*Ev*Baug + c(3)*(z0_'*Aaug'*Eth'-thref')*Q3aug*Eth*Baug]';
    
   
    % Ref
    
           
% %% The cost function
%     % The weight
%     c = [1,10,20];
%     % The distance metric between the original path and the new path
%     Q1 = eye(nstep*dim);
%     %Q1((nstep-1)*dim+1:end,(nstep-1)*dim+1:end) =  eye(dim)*1000;
%     %Q1(1:dim,1:dim) =  eye(dim)*1000;
%     % The velocity
%     Vdiff = eye(nstep*dim)-diag(ones(1,(nstep-1)*dim),dim);
%     Vconst = [-eye(2) eye(2) zeros(2,(nstep-2)*2);[[zeros((nstep-1)*2,2) eye((nstep-1)*2) ]-[eye((nstep-1)*2) zeros((nstep-1)*2,2)]]];
%     V_ratio = [5 0;0 0];
%     Rpenalty = kron(eye(nstep),V_ratio);
%     Q2 = Vconst'*Rpenalty'*Rpenalty*Vconst;
%     %Q2 = Vdiff(1:(nstep-1)*dim,:)'*Q1(1+dim:end,1+dim:end)*Vdiff(1:(nstep-1)*dim,:);
%     Vref = [xV,0]*dt;    
%     Vref_1 = c(2)*kron(ones(1, nstep),Vref)*Rpenalty'*Rpenalty*Vconst;
%     % The accelaration
%     Vdiff = eye(nstep*dim)-diag(ones(1,(nstep-1)*dim),dim);
%     Adiff = Vdiff-diag(ones(1,(nstep-1)*dim),dim)+diag(ones(1,(nstep-2)*dim),dim*2);
%     Q3 = Adiff(1:(nstep-2)*dim,:)'*Adiff(1:(nstep-2)*dim,:);   
% %% Cost function update
%     dir = [zT(1)-(-6) zT(2)-0];
%     dir_T = (1/norm(dir))*[ zT(2)-0 -zT(1)+(-6)];
%     dd = kron(ones(nstep,1),dir_T*[-6;0]);
%     D = kron(eye(nstep),dir_T);
%     % Distance to reference line
%     Q1 = D'*D;
%     Xdis_1 = 2*c(1)*dd'*D;
%     % The total costj
%     Qref = 1*(Q1*c(1)+Q2*c(2)+Q3*c(3));
%     Qabs = 0*Q3*c(3);



%% MM Constraints 
% AT = zeros(4*dim,nstep*dim);
% AT(0*dim+1:1*dim,1:dim) = eye(dim);
% AT(1*dim+1:2*dim,(nstep-1)*dim+1:nstep*dim) = eye(dim);
% AT(2*dim+1:3*dim,1:2*dim) = [-eye(dim) eye(dim)];
% AT(3*dim+1:4*dim,(nstep-2)*dim+1:nstep*dim) = [-eye(dim) eye(dim)];
% bT = [path(:,1);path(:,end);path(:,2)-path(:,1);path(:,end)-path(:,end-1)];
% AT = zeros(1*dim,nstep*dim);
% AT(0*dim+1:1*dim,1:dim) = eye(dim);      
% bT = [path(:,1)];
% 
% %% The Iteration
% refpath = [];
% for i=1:nstep
%     refpath = [refpath;path(:,i)];
% end
% oripath = refpath;
% refinput = ones(1,nstep);
% 
% % TB linear term
% %fT = [-Qref*oripath;];
% fT = [-[Xdis_1']-[Vref_1']];
% % TB Quadratic term
% QT = Qref+Qabs;
% poly_now(:,:,1)=obs{1}.poly;
% 
% %% For initialization on w 
% Aeq = zeros(1*dim,nstep*dim+nobj*nstep+(nstep-1)*2);
% Aeq(0*dim+1:1*dim,1:dim) = eye(dim);      
% beq = [path(:,1)]; 
% 
% 



for k = 1:10
%% The constraint
% TB
%     LT = []; ST = []; margin = 0.2;
%     for i=1:nstep
%         for j=1:nobj
%             poly = obs{j}.poly+obs{j}.v*ones(1,4)*dt*i;
%             [L,S,d] = d2poly(refpath((i-1)*dim+1:i*dim)',poly');
%             %LT = [LT;zeros(1,(i-1)*dim) L zeros(1,(nstep-i)*dim) zeros(1,nstep)];
%             LT = [LT;zeros(1,(i-1)*dim) L zeros(1,(nstep-i)*dim)  zeros(1,5*24) zeros(1,(i-1)*nobj+j-1) -1 zeros(1,nobj*(nstep+1-i)-j) zeros(1,H)];
%             ST = [ST;S-margin];
%             % Soft constraint
%             LT= [LT;zeros(1,nstep*2) zeros(1,5*24) zeros(1,(i-1)*nobj+j-1) -1 zeros(1,nobj*(nstep+1-i)-j) zeros(1,H)];
%             ST = [ST;0];
%         end
%         
%     end

% Arm

    D=obs_arm_r;
    LA=[];SA=[];
    LLA=[];
    SSA=[];
    I=[];
    rec_d = [];
    for i=1:H
        % provide base according to current 2D path 
        xy = xref(i*nstate+1:i*nstate+2);
        base = [xy' 0.1];
        % get reference theta
        theta=[xref_pre(6,i+1) xref_pre(12:2:18,i+1)']';
        [distance,linkid]=dist_arm_3D_Heu_hc(theta,DH(1:njoint,:),base,obs_arm,robot.cap);
        rec_d = [rec_d distance];
        
        I = [I;distance-D];
        ff = @(x) dist_arm_3D_Heu_hc(x,DH(1:njoint,:),base,obs_arm,robot.cap);
        Diff = num_jac(ff,theta); Diff = Diff';
        
        Bj=Baug((i-1)*nstate+1:i*nstate,1:H*nu);
        s=I(i)-Diff'*Bj(1:njoint,:)*uref;
        l=-Diff'*Bj(1:njoint,:);
        %ll = reshape(l,[5,H]);
        lL = [];
        for i_re = 1:H
            lL = [lL l((i_re-1)*5+1)*(r/d) -l((i_re-1)*5+1)*(r/d) l((i_re-1)*5+2:i_re*5)];
        end
        LLA = [LLA;lL];
        SSA = [SSA;s];
        
%         LA=[LA;  lL  ];
%         SA=[SA;s];
        
         % Soft constraint
        LA=[LA; [ lL  zeros(1,i-1) -1 zeros(1,H-i)]];
        SA=[SA;s];
        LA = [LA;[ zeros(1,H*6)  zeros(1,i-1) -1 zeros(1,H-i)]];
        SA = [SA;0];
    end
% TpA
    % Quadratic term
%     Q = blkdiag(QT,0.1*QA);
%     Q = blkdiag(Q,1000*diag(ones(1,nobj*nstep)),1000*diag(ones(1,H)));
%     % Linear term
%     f = [fT' 0.1*fA'  zeros(nobj*nstep+H,1)']';
%     % inequality 
%     LTpA = [LT;LA];
%     STpA = [ST;SA];
%     % equality
%     Awt = Aaug(1:5,:);
%     Bwt = Baug(1:5,:);
%     Aw1 = Aaug(1:nstate:end,:);
%     Bw1 = Baug(1:nstate:end,:);
%     AAxy = zeros(nstep,nstep*2);
%     AAu = eye(nstep);
%     AAw = [zeros(1,nu*H);-Bw1];
%     AA = [AT zeros(size(AT,1),5*24 )];
%     AA = [AA zeros(size(AA,1),nobj*nstep+H)];
%     BA = [bT;];
%     
%     LTpA = [LTpA; [AAxy  [zeros(1,nu*H);-Bw1] zeros(size(AAxy,1),nobj*nstep+H) ] ];
%     STpA = [STpA;0; -uref_(2:end)' + Aw1*xR(:,1)+0.1];
%     LTpA = [LTpA; [AAxy  [zeros(1,nu*H);Bw1] zeros(size(AAxy,1),nobj*nstep+H) ] ];
%     STpA = [STpA;0; uref_(2:end)' - Aw1*xR(:,1)+0.1];
%% QP

    % Quadratic term
    soft = 100;
    Q = blkdiag(QA,soft*eye(H));
    % Linear term
    f = [fA'  zeros(H,1)']';
    
    options =  optimoptions('quadprog','Display','off');
    soln = quadprog(Q,f,LA,SA,[],[],[],[],[],options);
%    soln = quadprog(QA,fA,LA,SA,[],[],[],[],[],options);
    % TB path update 
    %waypoints = soln(1:dim*nstep);
    %refinput = soln(dim*nstep+1:(dim+1)*nstep);
    %x_out = soln(1:2:2*25);
    %y_out = soln(2:2:2*25);
    %u_out = soln(2*25+1:3*25);
    %%
    alpha_out = soln(1:6*H);
    states_out = Aaug*z0_+Baug*alpha_out;
    states_out = [z0_ ;states_out];
    theta_out = [  z0_(6)  states_out(6:nstate:end)';
                   z0_(12)  states_out(12:nstate:end)';
                   z0_(14)  states_out(14:nstate:end)';
                   z0_(16)  states_out(16:nstate:end)';
                   z0_(18)  states_out(18:nstate:end)'];
               
    waypoints = [  z0_(1)  states_out(1:nstate:end)';
                   z0_(2)  states_out(2:nstate:end)'];
    
    
                    
    % Arm u update
    
    %unew = soln((dim)*nstep+1:end-(nobj*nstep+H));
    
    
%     xref=[];
%     for i=2:H+1
%         xR(:,i)=robot.A([1:njoint,6:5+njoint],[1:njoint,6:5+njoint])*xR(:,i-1)+robot.B([1:njoint,6:5+njoint],1:nu)*unew((i-2)*nu+1:(i-1)*nu);
%         xref=[xref;xR(:,i)];
%     end
    uref=alpha_out;
    
    
    
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
    
    %uref = [uref(2:end); uref(end)];
    
end
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

theta_implement = [theta_implement Ax_current];
end_implement = [end_implement  end_effector(:,2)];
traj_implement = [traj_implement X_out(6:7)'];
pla_implement = [pla_implement X_out(13:15)'];


    
%if steps == 3||steps == 8||steps == 13 ||steps == 18||steps == 23||steps == 28||steps == 33||steps == 38
end_ = plot3(end_effector(1,:),end_effector(2,:),end_effector(3,:),'-*','color',[1-(steps/ss),1-(steps/ss),1-(steps/ss)]);

hold on
traLQ_= plot(X_out(1,:),X_out(2,:),'-*','color',[1-(steps/ss),1-(steps/ss),1-(steps/ss)]);

hold on
%end
% ob = Polyhedron('V',obs{1}.poly');
% ob.plot('color','g');
% axis equal
% axis([-0.5 2.5 -.6 .5 -0.5 1.2]);
% r=0.14;
% [X,Y,Z] = cylinder(r);
% h=mesh(X+1.18,Y-.1,Z*0.5,'facecolor',[1 0 0]);
% %legend('end-effector','platform','obs. (for platform)','obs. (for arm)','location','eastoutside')
% view(3);
% %subplot(1,2,2);
% xlabel('x[m]')
% ylabel('y[m]')
% zlabel('z[m]')
% 
% %%%%%%%%%%%%%%%%
% %pause(0.05)
% delete(end_)
% %delete(traLQ_)

end

%%
figure(fighandle(1));
% ob = Polyhedron('V',obs{1}.poly');
% Obs = ob.plot('color','g');

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
% ref.x = x_out;
% ref.y = y_out;
% ref.v = [v_out v_out(end)];
% ref.w = u_out;

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

