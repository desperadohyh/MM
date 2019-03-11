% This script is to show how to map an obstacle from cartesian space to
% joint space of a robot arm by sampling
clc
clear

currfold = pwd;
addpath(strcat(currfold,'/Lib'));
addpath(strcat(currfold,'/DERIVESTsuite'));

robot=robotproperty(3);

% obstacle
% obs = [[4106;8313;1072]./1000 [4106;8313;1072]./1000];
obs = [4106;6313;5072]./1000;

% njoint=5;nstate=10;nu=5;DH=robot.DH;
%% map obs from cartesian space to configuration space 
% consider only the third arm 
% has the collision with obs

% robot para
% capsule
cap = robot.cap;
margin = 0;
% base pos
base = [3250, 8500,0]./1000;

% position of third arm
id_link = 3;
syms t1 t2 t3
DH = [];
for i = 1:id_link
    DH_rest = robot.DH(i,2:4);
    eval(['DH_tmp = [t' num2str(i) ' DH_rest]']); 
    DH = [DH; DH_tmp];
end
pos=SymCapPos(base',DH,cap, id_link);

%% obstacle mapping 
% cartesian space to configuration space

% [dis, points] = distLinSeg(pos.p(:,1),pos.p(:,2), obs(:,1),obs(:,2));
% sample = solve(dis == cap{3}.r, [t1, t2, t3]);
% inf_set = [sample.t1, sample.t2, sample.t3];
figure
inf_set = [];
for ratio = 0.1:0.1:1
    s_pos = pos.p(:,1) + ratio*(pos.p(:,2) - pos.p(:,1));
%     s_pos = pos.p(:,1);
%     dis = norm(s_pos - obs(:,1));
%     sample = solve(dis == cap{3}.r + margin, [t1, t2, t3]);
    sample = solve([s_pos(1) == obs(1), s_pos(2) == obs(2), s_pos(3) == obs(3)], [t1, t2, t3]);
    inf_tmp = [sample.t1 sample.t2 sample.t3];
    plot3(sample.t1, sample.t2, sample.t3, '*r');
    hold on
    inf_set = [inf_set;inf_tmp];
end












%% CFS Iteration
MAX_ITER = 5;

D=0.09;
for k=1:MAX_ITER
    tic
    Lstack=[];Sstack=[];
    for j=1:1
        I=[];
        for i=1:horizon
            theta=xref(nstate*(i-1)+1:nstate*(i-1)+njoint);
            [distance,linkid]=dist_arm_3D_Heu(theta,DH(1:njoint,:),base,obs,robot.cap);
            %distance=dist_arm_3D(theta,DH(1:njoint,:),base,obs,robot.cap);
            I = [I;distance-D];
            Diff=zeros(njoint,1);
            for s=1:njoint
                [Diff(s),~]=derivest(@(x) dist_link_Heu([theta(1:s-1);x;theta(s+1:end)],DH(1:njoint,:),base,obs,robot.cap,linkid),theta(s),'Vectorized','no');
            end
            Diff;
            %Hess=hessian(@(x) dist_link_Heu(x,DH(1:njoint,:),base,obs,robot.cap,linkid),theta);
            Bj=Baug((i-1)*nstate+1:i*nstate,1:horizon*nu);
            s=I(i)-Diff'*Bj(1:njoint,:)*uref;
            l=-Diff'*Bj(1:njoint,:);
            
            
            Sstack=[Sstack;s];
            Lstack=[Lstack;l];
            
        end
    end
    
    
    
end






















%% From joint to cartesian
% Show the arm in both the joint space and the cartesian space
disp('Point in the joint space: ')
disp(theta)

figure(1);
subplot(121);hold on;
plot(theta(1), theta(2), '*k')
title('Joint space')
xlabel('\theta_1')
ylabel('\theta_2')
axis equal
axis([-pi, pi, -pi, pi])

arm_config = zeros(ndim, nlink);
arm_config(:, 1) = origin;
for i = 1:nlink
    arm_config(:, i+1) = arm_config(:, i) + [cos(sum(theta(1:i)));sin(sum(theta(1:i)))]
end
subplot(122);hold on;
plot(arm_config(1, :), arm_config(2, :),'k')
title('Cartesian space')
xlabel('x position')
ylabel('y position')
axis equal

%% From cartesian to joint
% Map the obstacle from cartesian space to joint space
point = [1;1]; % point obstacle
subplot(122)
plot(point(1), point(2), 'or')

% Consider the case that collision only happens for the second link
syms t1 t2
obs = [];
subplot(121)
l2_min = norm(point)-link(1);
for l2 = l2_min:0.05:link(2)
    px = link(1)*cos(t1)+l2*cos(t1+t2);
    py = link(1)*sin(t1)+l2*sin(t1+t2);
    % solve([f(x) - a],x) means to calculate the solution for x such that f(x) = a 
    sample = solve([px-point(1), py-point(2)], t1, t2); % This step is very slow
    plot(sample.t1, sample.t2, '*r')
    obs = [obs [double(sample.t1)'; double(sample.t2)']];
end

% Generating convex hull of the obstacle in the joint space
k = convhull(obs(1,:), obs(2,:));
plot(obs(1, k),obs(2, k),'r-')


