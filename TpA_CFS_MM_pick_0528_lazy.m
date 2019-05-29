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
nobj        = 1;
obs         = {};
obs{1}.v    = [0;0];

xd = 0;
yd = -0.5;
obs{1}.poly = [1.1+xd 1.3+xd+0.5 1.4+xd+0.5 0.9+xd;0.1+yd 0.1+yd -0.5+yd -0.5+yd];
% TB: trajectory dimension
dim         = 2; %x,y
%%%%%%%%%%%%%%%%%%%%%%%%%%%
gr_pick
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Arm parameters
robot=robotproperty_hc(4);
% Arm joint
njoint      =5; % joint number
nstate      =10; % QP variable dim
nu          =5; % acceleration dim 
DH          =robot.DH;

% Arm obs
% center position (1.05,-0.2)
obs_arm     =[[1.05;-0.4;0.35] [1.05;-0.4;0.5]];
obs_arm_r   = 0.05; % radius


% Target
target = [0; 0; 0.05];
t_marg = [0.35 0.2; 0  0; 0 0];

ss=35;
xV = -0.15

% Initializa conditions
pass = 0;
mode = 1;


% for simulation testing
fail = 1;

% load grasping profile
grip_open = 0.01;
grip_close = 0.2;
g_size =10;

ccc = [load('get_far')];
theta_far = ccc.thetas;
ddd = [load('get_close')];
theta_close = ddd.thetas;

th_size = size(theta_close,2);

for steps=1:ss
%% Mode check and switch
% Check pass obs
if Tx_current(1) - 0.4 > 1.05
    pass = 1;
end

% Loop reference
[xref_t,xref,xR,refpath]=generate_reference_loop_pick(var,Ax_current,Tx_current,zAT,horizon,nstate,u0,mode,target,t_marg );


%% Arm Cost function and Constraints
if steps == 11||steps == 12
    yd=yd+0;
end
    obs{1}.poly = [1.1+xd 1.3+xd 1.4+xd 0.9+xd;0.1+yd 0.1+yd -0.5+yd -0.5+yd];
    
    




 
    
    
    % Cost Fn Parameters
    Aaug=[];Baug=zeros(horizon*nstate,horizon*nu);Qaug=zeros(horizon*nstate);
    Q_arm=[];
    Q_arm(1:njoint,1:njoint)=[1 0 0 0 0;
        0 10 0 0 0;
        0 0 5 0 0;
        0 0 0 2 0;
        0 0 0 0 2];
    Q_arm(1:njoint,njoint+1:2*njoint)=0.1*eye(njoint);
    Q_arm(njoint+1:2*njoint,1:njoint)=0.1*eye(njoint);
    Q_arm(njoint+1:2*njoint,njoint+1:2*njoint)=[1 0 0 0 0;
        0 10 0 0 0;
        0 0 1 0 0;
        0 0 0 1 0;
        0 0 0 0 1];
    for i=1:horizon
        Aaug=[Aaug;robot.A([1:njoint,6:5+njoint],[1:njoint,6:5+njoint])^i];
        Qaug((i-1)*nstate+1:i*nstate,(i-1)*nstate+1:i*nstate)=Q_arm*0.1;
        if i==10
            Qaug((i-1)*nstate+1:i*nstate,(i-1)*nstate+1:i*nstate)=Q_arm*10000;
        end
        for j=1:i
            Baug((i-1)*nstate+1:i*nstate,(j-1)*nu+1:j*nu)=robot.A([1:njoint,6:5+njoint],[1:njoint,6:5+njoint])^(i-j)*robot.B([1:njoint,6:5+njoint],1:nu);
        end
    end
    R=eye(horizon*nu);
    for i=1:horizon
        R((i-1)*nu+1:i*nu,(i-1)*nu+1:i*nu)=[5 0 0 0 0;
            0 4 1 0 0;
            0 1 2 0 0;
            0 0 0 2 0;
            0 0 0 0 1];
    end
    R=R+R';
    
    % Quadratic term
    QA =  Baug'*Qaug*Baug+R.*10;
    % Linear term
    fA = ((Aaug*xR(:,1)-xref)'*Qaug*Baug)';
    
   

%% The cost function
    % The weight
    c = [1,10,20];
    % The distance metric between the original path and the new path
    Q1 = eye(nstep*dim);
    % The velocity
    Vdiff = eye(nstep*dim)-diag(ones(1,(nstep-1)*dim),dim);
    Vconst = [-eye(2) eye(2) zeros(2,(nstep-2)*2);[[zeros((nstep-1)*2,2) eye((nstep-1)*2) ]-[eye((nstep-1)*2) zeros((nstep-1)*2,2)]]];
    V_ratio = [5 0;0 0];
    Rpenalty = kron(eye(nstep),V_ratio);
    Q2 = Vconst'*Rpenalty'*Rpenalty*Vconst;
    %Q2 = Vdiff(1:(nstep-1)*dim,:)'*Q1(1+dim:end,1+dim:end)*Vdiff(1:(nstep-1)*dim,:);
    Vref = [xV,0]*dt;    
    Vref_1 = c(2)*kron(ones(1, nstep),Vref)*Rpenalty'*Rpenalty*Vconst;
    % The accelaration
    Vdiff = eye(nstep*dim)-diag(ones(1,(nstep-1)*dim),dim);
    Adiff = Vdiff-diag(ones(1,(nstep-1)*dim),dim)+diag(ones(1,(nstep-2)*dim),dim*2);
    Q3 = Adiff(1:(nstep-2)*dim,:)'*Adiff(1:(nstep-2)*dim,:);   
%% Cost function update
    dir = [zT(1)-(-6) zT(2)-0];
    %dir = [zT(1)-Tx_current(1) zT(2)-Tx_current(2)];
    dir_T = (1/norm(dir))*[ zT(2)-0 -zT(1)+(-6)];
    dd = kron(ones(nstep,1),dir_T*[-6;0]);
    D = kron(eye(nstep),dir_T);
    % Distance to reference line
    Q1 = D'*D;
    Xdis_1 = 2*c(1)*dd'*D;
    % The total costj
    Qref = 1*(Q1*c(1)+Q2*c(2)+Q3*c(3));
    Qabs = 0*Q3*c(3);


%% Extended cost
Mcurv = eye(nstep);
Mcurv(nstep,nstep) = 5;
Vcurv = eye(nstep)-diag(ones(1,nstep-1),1);
Acurv = Vcurv-diag(ones(1,(nstep-1)),1)+diag(ones(1,(nstep-2)),2);
Qcurv = 5*Mcurv;%+Vcurv(1:nstep-1,:)'*Vcurv(1:nstep-1,:)+Acurv(1:(nstep-2),:)'*Acurv(1:(nstep-2),:);
%% The boundary constraint
AT = zeros(1*dim,nstep*dim);
AT(0*dim+1:1*dim,1:dim) = eye(dim);      
bT = [refpath(1:2)];

%% The Iteration
oripath = refpath;
refinput = ones(1,nstep);

% TB linear term
%fT = [-Qref*oripath;];
fT = [-[Xdis_1']-[Vref_1']];
% TB Quadratic term
QT = Qref+Qabs;
poly_now(:,:,1)=obs{1}.poly;

%% For initialization on w 
Aeq = zeros(1*dim,nstep*dim+nobj*nstep+(nstep-1)*2);
Aeq(0*dim+1:1*dim,1:dim) = eye(dim);      
beq = [refpath(1:2)]; 



 for k = 1:25
    FEAS = 1;
    %% The constraint
    Lstack = []; Sstack = []; margin = 0.2;
    
    % Soft speed constraints
    Lstack = [-Vdiff(1:end-2,:) zeros(2*(nstep-1),nstep) -eye((nstep-1)*2)];
    Sstack = ones(2*(nstep-1),1)*xV*dt;
    
    Lstack = [Lstack;zeros((nstep-1)*2,nstep*2) zeros(2*(nstep-1),nstep) -eye((nstep-1)*2)];
    Sstack = [Sstack;zeros((nstep-1)*2,1)];
    
    Lstack = [Lstack; Vdiff(1:end-2,:) zeros(2*(nstep-1),nstep) -eye((nstep-1)*2)];
    Sstack = [Sstack; ones(2*(nstep-1),1)*xV*dt];
    
    Lstack = [Lstack;zeros((nstep-1)*2,nstep*2) zeros(2*(nstep-1),nstep) -eye((nstep-1)*2)];
    Sstack = [Sstack;zeros((nstep-1)*2,1)];
    
    
    for i=1:nstep
        for j=1:1
            poly = poly_now(:,:,1)+obs{j}.v*ones(1,4)*dt*i;
            [L,S,d] = d2poly(refpath((i-1)*dim+1:i*dim)',poly');
            
            % Constraint
%             Lstack = [Lstack;zeros(1,(i-1)*dim) L zeros(1,(nstep-i)*dim) ];
%             Sstack = [Sstack;S-margin]; 

            % Soft constraint
            Lstack = [Lstack;zeros(1,(i-1)*dim) L zeros(1,(nstep-i)*dim)  zeros(1,(i-1)*nobj+j-1) -1 zeros(1,nobj*(nstep+1-i)-j+(nstep-1)*2)];
            Sstack = [Sstack;S-margin];             
            Lstack = [Lstack;zeros(1,nstep*2) zeros(1,(i-1)*nobj+j-1) -1 zeros(1,nobj*(nstep+1-i)-j+(nstep-1)*2)];
            Sstack = [Sstack;0];
            
        end
    end

    %% QP     
    % enlarge A ,f for soft constraint
    
    H = blkdiag(Qref+Qabs,1000*diag(ones(1,nobj*nstep+(nstep-1)*2)));
    f = [-[Xdis_1']-[Vref_1']; zeros(nobj*nstep+(nstep-1)*2,1)];
    %f = [-Qref*oripath; zeros(nobj*nstep,1)]';
    options =  optimoptions('quadprog','Display','off');
    soln = quadprog(H,f,Lstack,Sstack,Aeq,beq,[],[],[],options);
    pathnew = soln(1:dim*nstep);
    
    if norm(refpath-pathnew) < 0.0001
        break
    end
    refpath=pathnew;
    
 end
 
 

%% LQR tracking
    traj=zeros(2,NILQR);
    traj(1,:) = pathnew(1:dim:dim*NILQR)';
    traj(2,:) = pathnew(2:dim:dim*NILQR)';
    ILQRtraj = traj(:,1:NILQR);
    xref_=[ILQRtraj; xref_t'; zeros(2,NILQR)];
    
   
    
    L1 = zeros(7,NILQR);
    
    L2 = diag([100 100 0 0 0 1 1 ]);
    L2=reshape(repmat(L2,1,NILQR),7,7,NILQR);
    l=zeros(1,NILQR);
    u0 = zeros(2,NILQR-1);


    [ x_out, u, xbar, ubar, k, K, sigsu, A, B ] = MaxEntILQR( L2, L1, l, Tx_current, u0, var, xref_ );      
    
    uref_ = x_out(3:5:end); %%%%%%%%%%%%%%%%%%
    
    

 

%% Iterative Quadratic Program solve 
tic

for k = 1:10
%% The constraint
% TB
    LT = []; ST = []; margin = 0.2;
    for i=1:nstep
        for j=1:nobj
            poly = obs{j}.poly+obs{j}.v*ones(1,4)*dt*i;
            [L,S,d] = d2poly(refpath((i-1)*dim+1:i*dim)',poly');
            % Constraints
            %LT = [LT;zeros(1,(i-1)*dim) L zeros(1,(nstep-i)*dim) zeros(1,nstep)];
            
            % Soft constraint
            LT = [LT;zeros(1,(i-1)*dim) L zeros(1,(nstep-i)*dim)  zeros(1,5*24) zeros(1,(i-1)*nobj+j-1) -1 zeros(1,nobj*(nstep+1-i)-j) zeros(1,horizon)];
            ST = [ST;S-margin];            
            LT= [LT;zeros(1,nstep*2) zeros(1,5*24) zeros(1,(i-1)*nobj+j-1) -1 zeros(1,nobj*(nstep+1-i)-j) zeros(1,horizon)];
            ST = [ST;0];
            
        end
        
    end

% Arm

    D=obs_arm_r;
    LA=[];SA=[];
    LLA=[];
    SSA=[];
    I=[];
    rec_d = [];
    for i=1:horizon
        % provide base according to current 2D path 
        xy = refpath(i*2+1:(i+1)*2);
        base = [xy' 0.1];
        % get reference theta
        theta=xref(nstate*(i-1)+1:nstate*(i-1)+njoint);
        [distance,linkid]=dist_arm_3D_Heu_hc(theta,DH(1:njoint,:),base,obs_arm,robot.cap);
        rec_d = [rec_d distance];
        
        I = [I;distance-D];
        ff = @(x) dist_arm_3D_Heu_hc(x,DH(1:njoint,:),base,obs_arm,robot.cap);
        Diff = num_jac(ff,theta); Diff = Diff';
        
        Bj=Baug((i-1)*nstate+1:i*nstate,1:horizon*nu);
        s=I(i)-Diff'*Bj(1:njoint,:)*uref;
        l=-Diff'*Bj(1:njoint,:);
        LLA = [LLA;l];
        SSA = [SSA;s];
        % Soft constraint
        LA=[LA; [zeros(1,nstep*2) l zeros(1,nstep*nobj) zeros(1,i-1) -1 zeros(1,horizon-i)]];
        SA=[SA;s];  
         
        LA = [LA;[zeros(1,nstep*2) zeros(1,horizon*5) zeros(1,nstep*nobj) zeros(1,i-1) -1 zeros(1,horizon-i)]];
        SA = [SA;0];
        
    end
% TpA
    % Quadratic term
    Q = blkdiag(QT,0.1*QA);
    Q = blkdiag(Q,1000*diag(ones(1,nobj*nstep)),1000*diag(ones(1,horizon)));
    % Linear term
    f = [fT' 0.1*fA'  zeros(nobj*nstep+horizon,1)']';
    % inequality 
    LTpA = [LT;LA];
    STpA = [ST;SA];
    % equality
    Awt = Aaug(1:5,:);
    Bwt = Baug(1:5,:);
    Aw1 = Aaug(1:nstate:end,:);
    Bw1 = Baug(1:nstate:end,:);
    AAxy = zeros(nstep,nstep*2);
    AAu = eye(nstep);
    AAw = [zeros(1,nu*horizon);-Bw1];
    AA = [AT zeros(size(AT,1),5*24 )];
    AA = [AA zeros(size(AA,1),nobj*nstep+horizon)];
    BA = [bT;];
    
    % Inequality constraints for th1 angle
    LTpA = [LTpA; [AAxy  [zeros(1,nu*horizon);-Bw1] zeros(size(AAxy,1),nobj*nstep+horizon) ] ];
    STpA = [STpA;0; -uref_(2:end)' + Aw1*xR(:,1)+0.1];
    LTpA = [LTpA; [AAxy  [zeros(1,nu*horizon);Bw1] zeros(size(AAxy,1),nobj*nstep+horizon) ] ];
    STpA = [STpA;0; uref_(2:end)' - Aw1*xR(:,1)+0.1];
    
    
%% QP
    options =  optimoptions('quadprog','Display','off');
    soln = quadprog(Q,f,LTpA,STpA,AA,BA,[],[],[],options);
    % TB path update 
    pathnew = soln(1:dim*nstep);
    refinput = soln(dim*nstep+1:(dim+1)*nstep);
    x_out = soln(1:2:2*25);
    y_out = soln(2:2:2*25);
    %u_out = soln(2*25+1:3*25);
    alpha_out = soln(2*25+1:end-(nobj*nstep+horizon));
    states_out = Aaug*xR(:,1)+Baug*alpha_out;
    theta_out = [  Ax_current(1)  states_out(1:10:end)';
                   Ax_current(2)  states_out(2:10:end)';
                   Ax_current(3)  states_out(3:10:end)';
                   Ax_current(4)  states_out(4:10:end)';
                   Ax_current(5)  states_out(5:10:end)'];
    
    
                    
    % Arm u update
    
    unew = soln((dim)*nstep+1:end-(nobj*nstep+horizon));
    
    oldref=xref;
    xref=[];
    for i=2:horizon+1
        xR(:,i)=robot.A([1:njoint,6:5+njoint],[1:njoint,6:5+njoint])*xR(:,i-1)+robot.B([1:njoint,6:5+njoint],1:nu)*unew((i-2)*nu+1:(i-1)*nu);
        xref=[xref;xR(:,i)];
    end
    uref=unew;
    
    
    
    % break criteria 
    if norm(refpath-pathnew) < 0.01 && all(norm(xref-oldref)<0.01)
        pathall = [pathall pathnew]; 
        disp(['converged at step ',num2str(k)]);
        break
    end
    if norm(refpath-pathnew) < 0.1 && all(LLA*uref<SSA)
        pathall = [pathall pathnew];
        disp(strcat('Local Optimal Found at step ',num2str(k)));
        %break;
    end
    refpath = pathnew;
    
    %uref = [uref(2:end); uref(end)];
end
toc
%%

   
       

    traj=zeros(2,NILQR);
    traj(1,:) = pathnew(1:dim:dim*NILQR)';
    traj(2,:) = pathnew(2:dim:dim*NILQR)';
    ILQRtraj = traj(:,1:NILQR);
    xref_=[ILQRtraj; xref_t'; zeros(2,NILQR)];
    
    
    L1 = zeros(7,NILQR);
    
    L2 = diag([1000 1000 0 0 0 10 10 ]);
    L2=reshape(repmat(L2,1,NILQR),7,7,NILQR);
    l=zeros(1,NILQR);
    %u0 = zeros(2,NILQR-1);


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
    
    end_effector(:,j)=M{i}(1:3,1:3)*[0.08;0;0]+M{i}(1:3,4)+[x_out(j);y_out(j);0.1];

end
%%


figure(fighandle(1));
%%%%%%%%%%%%%%%%%%%%%%
Tx_current =  X_out(6:10)';
Ax_current =  states_out(11:15);

theta_implement = [theta_implement Ax_current];
end_implement = [end_implement  end_effector(:,2)];
traj_implement = [traj_implement X_out(11:12)'];
pla_implement = [pla_implement X_out(13:15)'];


    
if steps == 3||steps == 8||steps == 13 ||steps == 18||steps == 23||steps == 28||steps == 33||steps == 38
end_ = plot3(end_effector(1,:),end_effector(2,:),end_effector(3,:),'-*','color',[1-(steps/ss),1-(steps/ss),1-(steps/ss)])

hold on
traLQ_= plot(X_out(1,:),X_out(2,:),'-*','color',[1-(steps/ss),1-(steps/ss),1-(steps/ss)])

hold on
end
 
% %%%%%%%%%%%%%%%%
% %pause(0.05)
% delete(end_)
% %delete(traLQ_)

%%%%%%%%%%%%%%%%%%%%%%%%
%check_grasping
%%%%%%%%%%%%%%%%%%%%%%%%
%% check grasping
norm([Tx_current(1:2);0.05]-(target+t_marg(:,mode)))
if norm([Tx_current(1:2);0.05]-(target+t_marg(:,mode)))<0.02
    %%% Platform stop%%
    
    if mode == 1   % Ready to grasp and lift
    %g_current = 
        grip_ang = [linspace(grip_open,grip_close,g_size)]';
        ii = 0;
        %%%%% Subscribe gripper angle%%%
        v_grip = 0.2;
        
        while v_grip > 0.1 && ii<g_size+1
            grip_pub = grip_ang(ii);
            ii = ii+1;
            pause(0.1);
        end        
        grasp = 1;     
    
        %zB = [0;Ax_current(2);Ax_current(3)/2;Ax_current(4);Ax_current(5)];
        xref_lift = theta_far;
                     

         for j = 1:g_size
             pub = xref_lift(:,j);
             

             if  fail == 1%sensor>threshold
                 % FAIL: Play back and Open griper 
                 for b = j-1:-1:1
                     pub = xref_lift(:,j);                     
                 end
                 grasp = 0;
    %              for j = 1:g_size
    %                  pub = xref_lift(:,j);
    %              end
                 mode = 2;  % need to restart
%            fail = 0;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

             %%%%%%% lazy mode 2%%%%%%%%%%%%%%%%%%%%%%
             
             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                 break
             end
             mode = 3;  % Ready to move the object       
         end
         
    elseif mode == 3  % Ready to put down
        for b = g_size-1:-1:1
            pub = xref_lift(:,j);            
        end
        grasp = 0;
        mode = 1;  % Ready for new task
    
     
    end
    
    
end
end

%%
figure(fighandle(1));
ob = Polyhedron('V',obs{1}.poly');
Obs = ob.plot('color','g');
hold on
axis equal
axis([-0.5 3.5 -.6 .5 -0.5 1.2]);
r=0.14;
[X,Y,Z] = cylinder(r);
h=mesh(X+1.18,Y-.1,Z*0.5,'facecolor',[1 0 0]);
etrj = plot3(end_implement(1,:),end_implement(2,:),end_implement(3,:),'*r-')
ptrj = plot(traj_implement(1,:),traj_implement(2,:),'-or');

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