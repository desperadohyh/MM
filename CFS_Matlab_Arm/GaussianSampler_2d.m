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


for i = 1:10000
    % random sample a point p1
    p1 =  mvnrnd(mu,sigma,1);
    % random sample a piont p2 
    % around p1 within a gaussian distributed distance
    p2 = mvnrnd(p1,sigma_d,1);
    if ~chk_rng(p1, p2, limits)
        flag_cols = chk_cols_2d(p1,p2,obs,margin,link);
        if flag_cols(1) == 1 && flag_cols(2) == 0
            infSet = [infSet; p1];
        elseif flag_cols(1) == 0 && flag_cols(2) == 1
            infSet = [infSet;p2];
        end 
    end
end

rng default  % For reproducibility
figure
k = convhull(infSet(:,1), infSet(:,2));
toc
plot(infSet(:,1),infSet(:,2),'+')
hold on
plot(infSet(k, 1),infSet(k, 2),'r-')




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

function flag = chk_rng(p1, p2, limit)
    % check if sample is out of range
    flag = 0;
    for i = 1:size(p1,2)
        if p1(i) < limit(i,1) || p1(i) > limit(i,2) || p2(i) < limit(i,1) || p2(i) > limit(i,2)
            flag = 1;
        end
    end
end




