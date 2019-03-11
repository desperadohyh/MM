function [d, linkid] = dist_arm_3D_Heu(theta,DH,base,obs,cap)
% ????obstacle???arm????id

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
nlink=size(DH,1);
pos=cell(1,nlink);
M=cell(1,nlink+1); M{1}=eye(4);
for i=1:nlink
    R=[cos(DH(i,1)) -sin(DH(i,1))*cos(DH(i,4)) sin(DH(i,1))*sin(DH(i,4));
        sin(DH(i,1)) cos(DH(i,1))*cos(DH(i,4)) -cos(DH(i,1))*sin(DH(i,4));
        0  sin(DH(i,4)) cos(DH(i,4))];
    T=[DH(i,3)*cos(DH(i,1));DH(i,3)*sin(DH(i,1));DH(i,2)];
    M{i+1}=M{i}*[R T; zeros(1,3) 1];
    for k=1:2
        pos{i}.p(:,k)=M{i+1}(1:3,1:3)*RoCap{i}.p(:,k)+M{i+1}(1:3,4)+base;
    end
end

for i=1:nstate
    [dis, points] = distLinSeg(pos{i}.p(:,1),pos{i}.p(:,2), obs(:,1),obs(:,2));
    if norm(dis)<0.0001
        dis = -norm(points(:,1)-pos{i}.p(:,2));
    end        
    if dis < d
        d = dis;
        linkid=i;
    end
end
end