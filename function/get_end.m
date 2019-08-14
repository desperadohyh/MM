function [pend]=get_end(theta_,traj_,robot)

nstep = size(theta_,2);
pend = [];
T = robot.T;

for j = 1:nstep     
    
    theta=theta_(:,j);    
    
    nlink=size(robot.DH,1);
    pos=cell(1,nlink);
    M=cell(1,nlink+1); M{1}=eye(4);

    for i=2:nlink+1
        % R 
        R=[cos(theta(i-1)) -sin(theta(i-1))  0;
            sin(theta(i-1)) cos(theta(i-1))  0;
              0                        0                 1];
               
        if i == 4
            Rx=[1     0             0        ;  
                0  cos(-0.5*pi) -sin(-0.5*pi); 
                0  sin(-0.5*pi) cos(-0.5*pi) ];
                   
            R = Rx*R;
        end
        M{i}=M{i-1}*[R T(:,i); zeros(1,3) 1]; 
        
    end
    
    pend(:,j)=M{i}(1:3,1:3)*[0.08;0;0]+M{i}(1:3,4)+[traj_(1,j);traj_(2,j);0.1];

end


end
