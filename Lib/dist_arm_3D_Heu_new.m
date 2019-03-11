function [d, linkid] = dist_arm_3D_Heu_new(theta,DH,base,obs,cap)
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
DHn = [zeros(1,4);DH];
RoCap = cap;
nlink=size(DH,1);
pos=cell(1,nlink);
M=cell(1,nlink+1); M{1}=eye(4);
for i=2:nlink+1
        % R in book
        R=[cos(DHn(i,1)) -sin(DHn(i,1)) 0;
            sin(DHn(i,1))*cos(DHn(i-1,4)) cos(DHn(i,1))*cos(DHn(i-1,4)) -sin(DHn(i-1,4));
            sin(DHn(i,1))*sin(DHn(i-1,4)) cos(DHn(i,1))*sin(DHn(i-1,4))  cos(DHn(i-1,4))];
        % T in book
        T=[DHn(i-1,3);-sin(DHn(i-1,4))*DHn(i,2);cos(DHn(i-1,4))*DHn(i,2)];
        M{i}=M{i-1}*[R T; zeros(1,3) 1]; 
        for k=1:2
         pos{i-1}.p(:,k)=M{i}(1:3,1:3)*RoCap{i-1}.p(:,k)+M{i}(1:3,4)+base;
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