function [ pos,M] = plot_link(theta,base,cap)
% ????obstacle???arm????id
T = [0   0 -0.15    0   0.03 0.13;
     0   0   0      0  -0.12   0 ;
     0   0   0.0  0.034     0    0];
nstate = size(theta,1);
d = Inf;
if size(base,2)>1
    base=base';
end

%pos=CapPos(base,DH,cap);

% Ulternative for CapPos

RoCap = cap;
nlink=size(theta,1)
pos=cell(1,nlink);
M=cell(1,nlink+1); M{1}=eye(4);
for i=2:nlink+1
    RoCap{i-1}.p(:,3)=[0;0;0];
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
        for k=1:3
         pos{i-1}.p(:,k)=M{i}(1:3,1:3)*RoCap{i-1}.p(:,k)+M{i}(1:3,4)+base;
        end
end

pos{end+1}.p(:,3)=M{i}(1:3,1:3)*[0.08; 0; 0 ]+M{i}(1:3,4)+base;



end