clc
clear
close all
rosshutdown
% fighandle = [];
% fighandle(1) = figure(1); hold on;
% set(gcf, 'position', [0 0 500 500]);


%% ros

rosinit

odom_sub = rossubscriber('/om_with_tb3/odom','nav_msgs/Odometry');

odom_path_pub = rospublisher('path', 'nav_msgs/Path');
odom_path_msg = rosmessage('nav_msgs/Path');
odom_path_msg.Header.FrameId = 'odom';

joint_sub = rossubscriber('/om_with_tb3/joint_states', 'sensor_msgs/JointState');

joint1_pub = rospublisher('joint1_waypoints', 'std_msgs/Float64MultiArray');
joint2_pub = rospublisher('joint2_waypoints', 'std_msgs/Float64MultiArray');
joint3_pub = rospublisher('joint3_waypoints', 'std_msgs/Float64MultiArray');
joint4_pub = rospublisher('joint4_waypoints', 'std_msgs/Float64MultiArray');

joint1_msg = rosmessage('std_msgs/Float64MultiArray');
joint2_msg = rosmessage('std_msgs/Float64MultiArray');
joint3_msg = rosmessage('std_msgs/Float64MultiArray');
joint4_msg = rosmessage('std_msgs/Float64MultiArray');

r = robotics.Rate(0.5);

%% parameter definition

% TB definition
dt = 0.5;                                                                   % sampling time

% TB: obstacle
nobj = 1;                                                                   % number of obstacles
obs = {};
obs{1}.poly = [2.4 2.6 2.7 2.2; .1 .1 -.5 -.5];
obs{1}.v = [0; 0];
    
% TB: trajectory dimension
dim = 2;                                                                    % dimensions (x,y)

% TB: feasibility 
FEAS = 1;

% Arm parameters
robot = robotproperty_hc(4);

% Arm joint
njoint = 5;                                                                 % joint number
nstate = 10;                                                                % QP variable dim
nu = 5;                                                                     % acceleration dim 
DH = robot.DH;

% Arm base
base = [0 0 0];

% Arm obs
obs_arm = [[2.5;-0.2;0] [2.5;-0.2;0.5]];                                    % center position (1.05,-0.2)
obs_arm_r = 0.3;                                                            % radius

NILQR=25;
var.N = NILQR;
var.dt = dt;
var.nstep = 25;
var.nx = 5;
var.nu = 2;
var.Thres=1e-4;
var.lineSearchThres=1e-4;
generate_reference
xV = 0.1;

%% mpc

ss=100;
actual_traj = [];
mpc_traj = cell(0);

for steps=1:ss
    %% turtlebot sub
    odom_msg = receive(odom_sub, 10);
    euler = quat2eul([odom_msg.Pose.Pose.Orientation.W, ...
        odom_msg.Pose.Pose.Orientation.X, ...
        odom_msg.Pose.Pose.Orientation.Y, ...
        odom_msg.Pose.Pose.Orientation.Z]);
    
    Tx_current = [  odom_msg.Pose.Pose.Position.X;
        odom_msg.Pose.Pose.Position.Y;
        euler(3);
        odom_msg.Twist.Twist.Linear.X;
        odom_msg.Twist.Twist.Angular.Z];
    actual_traj = [actual_traj, Tx_current];
    
    %% arm sub
    joint_msg = receive(joint_sub, 10);
    Ax_current = [euler(3); joint_msg.Position(3:end)];
    
    %% obstacle sub
    
%     num_obstacles = sum(cell2mat(regexp(rostopic('list'), '/obstacle\d*_poly')));
%     obs = {};
%     for i = 1:num_obstacles
%         obstacle_poly_sub = rossubscriber(['/obstacle', num2str(i), '_poly'], 'geometry_msgs/PolygonStamped');
%         obstacle_v_sub = rossubscriber(['obstacle', num2str(i), '_v'], 'geometry_msgs/Twist');
%         obstacle_poly = receive(obstacle_poly_sub, 10);
%         obstacle_v = receive(obstacle_v_sub, 10);
%         obs{i}.v = [obstacle_v.Linear.X; obstacle_v.Linear.Y];
%         obs{i}.poly = [];
%         for j = 1:length(obstacle_poly.Polygon.Points)
%             obs{i}.poly = [obs{i}.poly, [double(obstacle_poly.Polygon.Points(j).X); double(obstacle_poly.Polygon.Points(j).Y)]];
%         end
%     end
    
    [xref_t,xori,xref,xR,path,refpath,uref]=generate_reference_loop(var,Ax_current,Tx_current,zAT,horizon,nstate,uref,u0 );
    
    %Tx_current %%%%%%%%%%%%%
    
    
    %% Arm Cost function and Constraints
     % Cost Fn Parameters
    Aaug=[];Baug=zeros(horizon*nstate,horizon*nu);Qaug=zeros(horizon*nstate);
    Q_arm=[];
    Q_arm(1:njoint,1:njoint)=[1 0 0 0 0;
        0 10 0 0 0;
        0 0 5 0 0;
        0 0 0 5 0;
        0 0 0 0 5];
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
        if i==horizon
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
    fA = ((Aaug*xR(:,1)-xori)'*Qaug*Baug)';
    
   

%% The cost function
    % The weight
    c = [1,10,1];
    % The distance metric between the original path and the new path
    Q1 = eye(nstep*dim);
    %Q1((nstep-1)*dim+1:end,(nstep-1)*dim+1:end) =  eye(dim)*1000;
    %Q1(1:dim,1:dim) =  eye(dim)*1000;
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
% AT = zeros(4*dim,nstep*dim);
% AT(0*dim+1:1*dim,1:dim) = eye(dim);
% AT(1*dim+1:2*dim,(nstep-1)*dim+1:nstep*dim) = eye(dim);
% AT(2*dim+1:3*dim,1:2*dim) = [-eye(dim) eye(dim)];
% AT(3*dim+1:4*dim,(nstep-2)*dim+1:nstep*dim) = [-eye(dim) eye(dim)];
% bT = [path(:,1);path(:,end);path(:,2)-path(:,1);path(:,end)-path(:,end-1)];
AT = zeros(1*dim,nstep*dim);
AT(0*dim+1:1*dim,1:dim) = eye(dim);      
bT = [path(:,1)];

%% The Iteration
refpath = [];
for i=1:nstep
    refpath = [refpath;path(:,i)];
end
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
beq = [path(:,1)]; 



 for k = 1:25
    FEAS = 1;
    %% The constraint
    Lstack = []; Sstack = []; margin = 0.2;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Lstack = [-Vdiff(1:end-2,:) zeros(2*(nstep-1),nstep) -eye((nstep-1)*2)];
    Sstack = ones(2*(nstep-1),1)*xV*dt;
    
    Lstack = [Lstack;zeros((nstep-1)*2,nstep*2) zeros(2*(nstep-1),nstep) -eye((nstep-1)*2)];
    Sstack = [Sstack;zeros((nstep-1)*2,1)];
    
    Lstack = [Lstack; Vdiff(1:end-2,:) zeros(2*(nstep-1),nstep) -eye((nstep-1)*2)];
    Sstack = [Sstack; ones(2*(nstep-1),1)*xV*dt];
    
    Lstack = [Lstack;zeros((nstep-1)*2,nstep*2) zeros(2*(nstep-1),nstep) -eye((nstep-1)*2)];
    Sstack = [Sstack;zeros((nstep-1)*2,1)];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for i=1:nstep
        for j=1:1
            poly = poly_now(:,:,1)+obs{j}.v*ones(1,4)*dt*i;
            [L,S,d] = d2poly(refpath((i-1)*dim+1:i*dim)',poly');
%             Lstack = [Lstack;zeros(1,(i-1)*dim) L zeros(1,(nstep-i)*dim) ];
%             Sstack = [Sstack;S-margin]; 
            Lstack = [Lstack;zeros(1,(i-1)*dim) L zeros(1,(nstep-i)*dim)  zeros(1,(i-1)*nobj+j-1) -1 zeros(1,nobj*(nstep+1-i)-j+(nstep-1)*2)];
            Sstack = [Sstack;S-margin]; 
            % Soft constraint
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
 
%if X_out(1)>1.5

% figure(fighandle(2));
% plot(pathnew(1:dim:end),pathnew(2:dim:end),'-*')
% %refpath(10:dim:end)=0;
% hold on
% %plot(x_out(1:5:end),x_out(2:5:end),'-*')
% ob = Polyhedron('V',obs{1}.poly');
% ob.plot('color','g');
% %end 
%  

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
    LT = []; ST = []; margin = 0.3;
    for i=1:nstep
        for j=1:nobj
            poly = obs{j}.poly+obs{j}.v*ones(1,4)*dt*i;
            [L,S,d] = d2poly(refpath((i-1)*dim+1:i*dim)',poly');
            %LT = [LT;zeros(1,(i-1)*dim) L zeros(1,(nstep-i)*dim) zeros(1,nstep)];
            LT = [LT;zeros(1,(i-1)*dim) L zeros(1,(nstep-i)*dim)  zeros(1,5*24) zeros(1,(i-1)*nobj+j-1) -1 zeros(1,nobj*(nstep+1-i)-j) zeros(1,horizon)];
            ST = [ST;S-margin];
            % Soft constraint
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
        LA=[LA; [zeros(1,nstep*2) l zeros(1,nstep*nobj) zeros(1,i-1) -1 zeros(1,horizon-i)]];
        SA=[SA;s];
        
         % Soft constraint
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

    poseArray = [];
    for i = 1:nstep
        poses = rosmessage('geometry_msgs/PoseStamped');
        poses.Pose.Position.X = x_out(i);
        poses.Pose.Position.Y = y_out(i);
        poseArray = [poseArray, poses];
    end
    odom_path_msg.Poses = poseArray;
    send(odom_path_pub, odom_path_msg);
    
    x_out_ed = x_out(end)
    y_out_ed = y_out(end)
    
    
    
    angles = reshape(states_out, 10, []);
    joint1_msg.Data = angles(2,:);
    joint2_msg.Data = angles(3,:);
    joint3_msg.Data = angles(4,:);
    joint4_msg.Data = angles(5,:);
    
    send(joint1_pub, joint1_msg);
    send(joint2_pub, joint2_msg);
    send(joint3_pub, joint3_msg);
    send(joint4_pub, joint4_msg);
%         
    %%%%%%%%%%%%%%%%%%%%%%
    % Tx_current =  X_out(6:10)';
    % Ax_current =  states_out(11:15);
    %
%     theta_implement = [theta_implement Ax_current];
%     end_implement = [end_implement  end_effector(:,2)];
%     traj_implement = [traj_implement X_out(11:12)'];
%     mpc_traj{end+1} = [x_out'; y_out'];
%     pla_implement = [pla_implement X_out(13:15)'];
    
    
    
    
    % end_ = plot3(end_effector(1,:),end_effector(2,:),end_effector(3,:),'*b-')
    %
    % hold on
    % traLQ_ = plot(X_out(1,:),X_out(2,:),'-*','color',[1-(steps/ss),1-(steps/ss),1-(steps/ss)])
    %
    % hold on
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
    waitfor(r)
end

%%
% figure(fighandle(1));
% ob = Polyhedron('V',obs{1}.poly');
% ob.plot('color','g');
hold on
axis equal
axis([-0.5 3.5 -.6 1.5 -0.5 1.2]);
r=0.14;
[X,Y,Z] = cylinder(r);
h=mesh(X+2.5,Y-.1,Z*0.5,'facecolor',[1 0 0]);
plot3(end_implement(1,:),end_implement(2,:),end_implement(3,:),'*r-')
plot(traj_implement(1,:),traj_implement(2,:),'-or');
plot(actual_traj(1,:), actual_traj(2,:), '.-b');

%legend('end-effector','platform','obs. (for platform)','obs. (for arm)','location','eastoutside')
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
%     figure
%     yyaxis left
%     plot(ref.v,'-bx')
%     %ylabel('Linear velocity [m/s]')
%     hold on
%     yyaxis right
%     plot(ref.w,'-ro')
%     %ylabel('Angular velocity [rad/s]')
%     legend('Vel','AngVel','location','best')
%     %xlabel('Time step')
    
    %%
%     figure
%     plot(ref.theta2,'m-d')
%     hold on
%     plot(ref.theta3,'--x')
%     plot(ref.theta4,'-r*')
%     plot(ref.theta5,'-go')
%     legend('\theta_1','\theta_2','\theta_3','\theta_4','location','eastoutside')
%     ylabel('Angle[rad]')
%     xlabel('Time step')
end

% %%
% figure
% plot(theta_implement(1,:),'-ko')
% hold on
% plot(theta_implement(2,:),'m-d')
% 
% plot(theta_implement(3,:),'--x')
% plot(theta_implement(4,:),'-r*')
% plot(theta_implement(5,:),'-go')
% legend('\theta_1','\theta_2','\theta_3','\theta_4','location','eastoutside')
% ylabel('Angle[rad]')
% xlabel('Time step')
% 
% ref.v = pla_implement(2,:);
% ref.w = pla_implement(3,:);
% 
% figure
% yyaxis left
% plot(ref.v,'-bx')
% %ylabel('Linear velocity [m/s]')
% hold on
% yyaxis right
% plot(ref.w,'-ro')
% %ylabel('Angular velocity [rad/s]')
% legend('Vel','AngVel','location','best')
% %xlabel('Time step')