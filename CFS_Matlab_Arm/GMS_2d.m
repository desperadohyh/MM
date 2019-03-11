% Gradient Mapping Sampler
%% Gaussian Sampler 
% 2D configuration space infeasible set
clc
clear
infSet = [];
tic
% Gaussian Sampling Process
link = [1;1];
mu = [pi/4, 0];
sigma = [pi/4 0; 0 pi/3];
sigma_d = [pi/15 0; 0 pi/15];
obs = [1 1];
margin = 0.1;

limits = [0, pi/2; -pi, pi]; % each row define the limits for one freedom

offset = [0 -pi];
scale = [pi/2 2*pi];
NP = [];



for i = 1:1000
    p1 = [];
    p1_base = rand(1,2);
    for j = 1:2
        tmp = scale(j)*p1_base(j) + offset(j);
        p1 = [p1 tmp];
    end
    if chk_cols_2d_single(p1,obs,margin,link)
        NP = [NP;p1];
    end
end

% generate direction
NP_vec_dir = [];
NP_vec = [];
vec_num = 20;
sigma_vec = [1 0; 0 1];
for t = 1:size(NP, 1)
    vec_set = []
    vec_dir_set = [];
    for m = 1:vec_num
        vec_dir = mvnrnd([0 0],sigma_vec,1);
        vec_dir = vec_dir/norm(vec_dir);
        vec_dir_set = [vec_dir_set;vec_dir];
        
        vec = NP(t,:);
        vec_set = [vec_set;vec];
    end
    NP_vec_dir = [NP_vec_dir {vec_dir_set}];
    NP_vec = [NP_vec {vec_set}];
end

eta = 1;
for iter = 1:20
    for n = 1:size(NP, 1)
        NP_vec_tmp = cell2mat(NP_vec(n));
        NP_vec_dir_tmp = cell2mat(NP_vec_dir(n));
        for v = 1:vec_num
            % update NP_vec
            NP_vec_tmp_i = NP_vec_tmp(v,:);
            NP_vec_dir_tmp_i = NP_vec_dir_tmp(v,:);
            [flag, dis] = chk_cols_2d_dis(NP_vec_tmp_i,obs,margin,link);
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


% plot
rng default 
figure
for i = 1:size(NP,1)
    plot(NP(i,1),NP(i,2),'*');
    hold on
    % plot convex hull for each NP vector set
    NP_vec_tmp = cell2mat(NP_vec(i));
    k = convhull(NP_vec_tmp(:,1), NP_vec_tmp(:,2));
    plot(NP_vec_tmp(k, 1),NP_vec_tmp(k, 2),'r-');
    hold on
end



function [flag, dis] = chk_cols_2d_dis(p1,obs,margin,link)
% cartesian space
    flag = 0;
    pos1 = [link(1)*cos(p1(1)) link(1)*sin(p1(1)); link(1)*cos(p1(1))+link(2)*cos(p1(1)+p1(2)) link(1)*sin(p1(1))+link(2)*sin(p1(1)+p1(2))];
    dis1 = distLinSeg(pos1(1,:), pos1(2,:), obs, obs);
    if dis1 <= margin
        flag = 1;
    end
    dis = abs(margin - dis1);
end

function flag = chk_cols_2d_single(p1,obs,margin,link)
% cartesian space
    pos1 = [link(1)*cos(p1(1)) link(1)*sin(p1(1)); link(1)*cos(p1(1))+link(2)*cos(p1(1)+p1(2)) link(1)*sin(p1(1))+link(2)*sin(p1(1)+p1(2))];
    dis1 = distLinSeg(pos1(1,:), pos1(2,:), obs, obs);
    if dis1 <= margin
        flag = 1;
    else
        flag = 0;
    end       
end



function flag = chk_cols_2d(p1,p2,obs,margin,link)
% cartesian space
    pos1 = [link(1)*cos(p1(1)) link(1)*sin(p1(1)); link(1)*cos(p1(1))+link(2)*cos(p1(1)+p1(2)) link(1)*sin(p1(1))+link(2)*sin(p1(1)+p1(2))];
    pos2 = [link(1)*cos(p2(1)) link(1)*sin(p2(1)); link(1)*cos(p2(1))+link(2)*cos(p2(1)+p2(2)) link(1)*sin(p2(1))+link(2)*sin(p2(1)+p2(2))];
    dis1 = distLinSeg(pos1(1,:), pos1(2,:), obs, obs);
    dis2 = distLinSeg(pos2(1,:), pos2(2,:), obs, obs);
    if dis1 <= margin
        flag1 = 1;
    else
        flag1 = 0;
    end
    if dis2 <= margin
        flag2 = 1;
    else
        flag2 = 0;
    end
    flag = [flag1 flag2];
end





