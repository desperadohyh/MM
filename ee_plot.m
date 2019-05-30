function end_effector = ee_plot(theta_implement, X_out,DH)
end_effector = [];
T = [0   0 -0.15    0   0.03 0.13;
     0   0   0      0  -0.12   0 ;
     0   0   0.0    0     0    0];

     
    
    theta = theta_implement(:,end);
    DHn = [zeros(1,4);DH];
    
    nlink=size(DH,1);
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
    
    end_effector=M{i}(1:3,1:3)*[0.08;0;0]+M{i}(1:3,4)+[X_out(1);X_out(2);0.1];

