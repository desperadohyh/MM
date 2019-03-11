% Gradient Mapping Sampler 
% 3D configuration space infeasible set
clc
clear
currfold = pwd;
addpath(strcat(currfold,'/Lib'));
addpath(strcat(currfold,'/DERIVESTsuite'));

robot=robotproperty(3);


% njoint=5;nstate=10;nu=5;DH=robot.DH;
%% map obs from cartesian space to configuration space 
% consider only the third arm 
% has the collision with obs

% robot para
% capsule
cap = robot.cap;
margin = 0;
% base pos
base = [3250,8500,0]./1000;

% position of third arm
id_link = 3;
limits = robot.thetamax;
obs = [4106;6313;5072]./1000;
margin = 4.25;

% infeasible set in configuration space
infSet = [];
tic

offset = [-pi 0 -pi];
scale = [2*pi pi 2*pi];
NP = [];

for i = 1:3000
    p1 = [];
    p1_base = rand(1,3);
    for j = 1:3
        tmp = scale(j)*p1_base(j) + offset(j);
        p1 = [p1 tmp];
    end
    DH = [p1' robot.DH(1:3,2:4)];
    % position for 3rd arm in Cartesian space
    pos1 = CapPos3(base',DH,cap,id_link);
    if chk_cols_3d_single(pos1,obs,margin)
        NP = [NP;p1];
    end
end

% generate direction
NP_vec_dir = [];
NP_vec = [];
vec_num = 10;
sigma_vec = [1 0 0;0 1 0;0 0 1];
for t = 1:size(NP, 1)
    vec_set = []
    vec_dir_set = [];
    for m = 1:vec_num
        vec_dir = mvnrnd([0 0 0],sigma_vec,1);
        vec_dir = vec_dir/norm(vec_dir);
        vec_dir_set = [vec_dir_set;vec_dir];
        
        vec = NP(t,:);
        vec_set = [vec_set;vec];
    end
    NP_vec_dir = [NP_vec_dir {vec_dir_set}];
    NP_vec = [NP_vec {vec_set}];
end

eta = 3;
for iter = 1:30
    for n = 1:size(NP, 1)
        NP_vec_tmp = cell2mat(NP_vec(n));
        NP_vec_dir_tmp = cell2mat(NP_vec_dir(n));
        for v = 1:vec_num
            % update NP_vec
            NP_vec_tmp_i = NP_vec_tmp(v,:);
            NP_vec_dir_tmp_i = NP_vec_dir_tmp(v,:);
            
            DH = [NP_vec_tmp_i' robot.DH(1:3,2:4)];
            % position for 3rd arm in Cartesian space
            pos = CapPos3(base',DH,cap,id_link);
            
            [flag, dis] = chk_cols_3d_dis(pos,obs,margin);
            if flag == 1
                % infeasible set
                NP_vec_tmp(v,:) = NP_vec_tmp_i + eta*dis*NP_vec_dir_tmp_i;
            else
                % feasible set
                NP_vec_tmp(v,:) = NP_vec_tmp_i - eta*dis*NP_vec_dir_tmp_i;
            end
        end
        % update NP_vec
        NP_vec(n) = {NP_vec_tmp};
    end
end
toc

% plot
rng default 
figure
for i = 1:size(NP,1)
    plot3(NP(i,1),NP(i,2),NP(i,3),'*');
    hold on
    % plot convex hull for each NP vector set
    NP_vec_tmp = cell2mat(NP_vec(i));
    k = convhull(NP_vec_tmp(:,1), NP_vec_tmp(:,2), NP_vec_tmp(:,3));
    plot3(NP_vec_tmp(k, 1),NP_vec_tmp(k, 2),NP_vec_tmp(k, 3),'r-');
    hold on
end
view(0,90)



function [flag, dis] = chk_cols_3d_dis(pos1,obs,margin)
% cartesian space
    flag = 0;
    pos1 = cell2mat(struct2cell(pos1));
    dis1 = distLinSeg(pos1(:,1),pos1(:,2),obs,obs);
    if dis1 <= margin
        flag = 1;
    end
    dis = abs(margin - dis1);
end




function flag = chk_cols_3d_single(pos1,obs,margin)
    pos1 = cell2mat(struct2cell(pos1));
    dis1 = distLinSeg(pos1(:,1),pos1(:,2),obs,obs);
    if dis1 <= margin
        flag = 1;
    else
        flag = 0;
    end
end
