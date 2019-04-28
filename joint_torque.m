function [d, linkid] = joint_torque(nlink,robot,DH,base,obs,cap)

theta = robot.theta;
R_ = robot.R_;
M = robot.M;
nlink = robot.nlink;
T = robot.T;
Tc = robot.Tc;
l = robot.l;
lc = robot.lc;
m = robot.m;
Z0 = robot.Z0;
R_inv = cell(1,nlink);

T = [0   0 -0.15    0   0.03 0.13;
     0   0   0      0  -0.12   0 ;
     0   0   0.0  0.04     0    0];

Vo_l = zeros(3,nlink);
Vo_g = zeros(3,nlink);
Wo_l = zeros(3,nlink);
Vc = zeros(3,nlink);
Keng = zeros(nlink,1);


Wo_l(:,1) = Z0(7);

for i = 1:nlink
    R_inv{i} = inv(R_{i});
end

Vc(:,1) = [ Z0(9)+(lc(1)*Z0(7)*sin(Z0(6))  Z0(11)-(lc(1)*Z0(7)*cos(Z0(6))  0]';
Keng(1) = 0.5*m(1)*Vc(:,1)'*Vc(:,1) + 0.5*Wo_l(:,1)'*Ic{1}*W0_l(:,1);

for i = 1:nlink-1
    Vo_l(:,i+1) = R_inv{i} * (Vo_l(:,i) + cross(Wo_l(:,i),T(:,i+2)));
    Vo_g(:,i+1) = M{i+2}(1:3,1:3)*Vo_l(:,i+1);
    Wo_l(:,i+1) = R_inv{i+1}*Wo_l(:,i) + [ 0 0 Z0(11+i*2)]';
    Vc(:,i+1) = Vo_g(:,i+1) + M{i+2}(1:3,1:3)*(cross(Wo_l(:,i+1),Tc(:,i+i)));
    Keng(i+1) = 0.5*m(i+1)*Vc(:,i+1)'*Vc(:,i+1) + 0.5*Wo_l(:,i+1)'*Ic{i+1}*W0_l(:,i+1);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nstate = size(DH,1);
for i=1:nstate
DH(i,1)=theta(i);
end%theta,d,a,alpha
d = Inf;
if size(base,2)>1
    base=base';
end

%pos=CapPos(base,DH,cap);

% Ulternative for CapPos

RoCap = cap;
nlink=size(theta,1);
pos=cell(1,nlink);
M=cell(1,nlink+1); M{1}=eye(4);
for i=2:nlink+1
        % R in book
        R=[cos(theta(i-1)) -sin(theta(i-1))  0;
            sin(theta(i-1)) cos(theta(i-1))  0;
              0                        0                 1];
        % T in book
        %T=[DHn(i-1,3);-sin(DHn(i-1,4))*DHn(i,2);cos(DHn(i-1,4))*DHn(i,2)];
        
        if i == 4
            Rx=[1     0             0        ;  
                0  cos(-0.5*pi) -sin(-0.5*pi); 
                0  sin(-0.5*pi) cos(-0.5*pi) ];
                   
            R = Rx*R;
        end
        M{i}=M{i-1}*[R T(:,i); zeros(1,3) 1]; 
        for k=1:2
         pos{i-1}.p(:,k)=M{i}(1:3,1:3)*RoCap{i-1}.p(:,k)+M{i}(1:3,4)+base;
        end
end

for i=1:nstate
    [dis, points] = distLinSeg(pos{i}.p(:,1),pos{i}.p(:,2), obs(:,1),obs(:,2));
    if norm(dis)<0.0001
        dis = -norm(points(1:3,1)-pos{i}.p(:,2));%%%%%%%%
    end        
    if dis < d
        d = dis;
        linkid=i;
    end
end
end