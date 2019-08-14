% door sys
r = 0.3;

Z0 = sym('z',[2 1]);
z0_ = sym('zk',[2 1]);
uk = sym('uk',[6 1]);

% J
Jk = [ -r*sin(Z0(1));
       r*cos(Z0(1)) ;
          0        ];
Jkf =  matlabFunction(Jk,'File','Jk_f');

% dJ
dJk = [ -r*cos(Z0(1))*Z0(2);
       -r*sin(Z0(1))*Z0(2) ;
                0        ];
            
dJkf =  matlabFunction(dJk,'File','dJk_f');
      
      
R_wc = [cos(Z0(1))  -sin(Z0(1))  0;
        sin(Z0(1))  cos(Z0(1))   0;
        0              0         1];

  
R_z90 = [cos(pi/2)  -sin(pi/2)  0;
        sin(pi/2)  cos(pi/2)   0;
        0              0         1];
    
    
R_x270 = [1       0             0 ;
          0     cos(-pi/2)    -sin(-pi/2);
          0     sin(-pi/2)     cos(-pi/2)];
  

Rgl =inv(R_wc*R_z90*R_x270);

Rglf =  matlabFunction(Rgl,'File','Rgl_f');
