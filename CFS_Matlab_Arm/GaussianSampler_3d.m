%% Gaussian Sampler 
% This script is to show how to map an obstacle from cartesian space to
% joint space of a robot arm by Gaussian Sampler
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
mu = [0 pi/2 0];
sigma = [pi/2 0 0;0 pi/4 0;0 0 pi/2];
sig_scale = 1/1000;
sigma_d = [pi*sig_scale 0 0;0 pi*sig_scale 0;0 0 pi*sig_scale];
obs = [4106;6313;5072]./1000;
margin = 4.25;

% infeasible set in configuration space
infSet = [];


tic
for i = 1:10000
    % Gaussian Distributed sample 
%     p1 =  mvnrnd(mu,sigma,1);
    % Uniform Distributed sample
    offset = [-pi 0 -pi];
    scale = [2*pi pi 2*pi];
    p1_base = rand(1,3);
    p1 = [];
    for j = 1:3
        tmp = scale(j)*p1_base(j) + offset(j);
        p1 = [p1 tmp];
    end
    
    
    % random sample a piont p2 
    % around p1 within a gaussian
    % distributed distance
    p2 = mvnrnd(p1,sigma_d,1);
    if ~chk_rng(p1,p2,limits)
        DH1 = [];
        DH2 = [];
        for i = 1:id_link
            DH_rest = robot.DH(i,2:4);
            DH1_tmp = [p1(i) DH_rest];
            DH2_tmp = [p2(i) DH_rest];
            DH1 = [DH1;DH1_tmp];
            DH2 = [DH2;DH2_tmp];
        end
        % position for 3rd arm in Cartesian space
        pos1 = CapPos3(base',DH1,cap,id_link);
        pos2 = CapPos3(base',DH2,cap,id_link);
        % check the potential collision with obstacle 
        % Gaussian Sampler
        flag_cols = chk_cols_3d(pos1,pos2,obs,margin);
        if flag_cols(1) == 1 && flag_cols(2) == 0
            infSet = [infSet; p1];
        elseif flag_cols(1) == 0 && flag_cols(2) == 1
            infSet = [infSet;p2];
        end  
        
        % normal sampler
%         flag = chk_cols_3d_single(pos1,obs,margin);
%         if flag == 1
%             infSet = [infSet; p1];
%         end
    end
end

toc
rng default  % For reproducibility
figure
plot3(infSet(:,1),infSet(:,2),infSet(:,3),'+')
hold on

k = convhull(infSet(:,1), infSet(:,2), infSet(:,3));

% view(0,90) % view through z axis direction

plot3(infSet(k, 1),infSet(k, 2),infSet(k,3),'r-')



function flag = chk_cols_3d_single(pos1,obs,margin)
    pos1 = cell2mat(struct2cell(pos1));
    dis1 = distLinSeg(pos1(:,1),pos1(:,2),obs,obs);
    if dis1 <= margin
        flag = 1;
    else
        flag = 0;
    end
end

function flag = chk_cols_3d(pos1,pos2,obs,margin)
    pos1 = cell2mat(struct2cell(pos1));
    pos2 = cell2mat(struct2cell(pos2));
    dis1 = distLinSeg(pos1(:,1),pos1(:,2),obs,obs);
    dis2 = distLinSeg(pos2(:,1),pos2(:,2),obs,obs);
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


