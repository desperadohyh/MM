function [ pos,M] = plot_link_4(T,theta,base,cap,flip)

nstate = size(theta,1);
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
    RoCap{i-1}.p(:,3)=[0;0;0];
        % R in book
        R=[cos(theta(i-1)) -sin(theta(i-1))  0;
            sin(theta(i-1)) cos(theta(i-1))  0;
              0                        0                 1];
        % T in book
        %T=[DHn(i-1,3);-sin(DHn(i-1,4))*DHn(i,2);cos(DHn(i-1,4))*DHn(i,2)];
        
        if i == 3
            Rz = eye(3);
            angle_f = 0.5*pi*(-1)^flip;
            Rx=[1     0             0        ;  
                0  cos(angle_f) -sin(angle_f); 
                0  sin(angle_f) cos(angle_f) ];
            if flip == 2
                Rz=[cos(angle_f)  -sin(angle_f) 0; 
                    sin(angle_f)  cos(angle_f)  0;  
                      0              0          1];
            end
            R = Rx*Rz*R;
        end
        
        M{i}=M{i-1}*[R T(:,i); zeros(1,3) 1]; 
        for k=1:3
         pos{i-1}.p(:,k)=M{i}(1:3,1:3)*RoCap{i-1}.p(:,k)+M{i}(1:3,4)+base;
        end
end

pos{end+1}.p(:,3)=M{i}(1:3,1:3)*[0.1; 0; 0 ]+M{i}(1:3,4)+base;



end