%% test torque


% robot parameters

%%%%%%%%%%%%%%%%%%%%%%%%%%%
gen_ref_MMD
robot=robotproperty_MMD_s(4, z0_, Ts);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
m = robot.m;
lc = robot.lc;
l = robot.l;
Ic = robot.Ic;

% generat data


Z0 = sym('z',[4 1]);
robot.z0_ = [ 0 0 -pi/2 0 ]';

M_11 = m(1)*lc(1)^2+Ic{1}(3,3)+m(2)*l(1)^2+m(2)*lc(2)^2+Ic{2}(3,3)+2*m(2)*l(1)*lc(2)*cos(Z0(3));
M_12 = m(2)*lc(2)^2+Ic{2}(3,3)+m(2)*l(1)*lc(2)*cos(Z0(3));
M_21 = m(2)*lc(2)^2+Ic{2}(3,3)+m(2)*l(1)*lc(2)*cos(Z0(3));
M_22 = m(2)*lc(2)^2+Ic{2}(3,3);

M = [M_11 M_12;
     M_21 M_22];
 
V = [ -2*m(2)*l(1)*lc(2)*Z0(2)*sin(Z0(3))*Z0(4)-m(2)*l(1)*lc(2)*sin(Z0(3))*Z0(4)^2;
                         m(2)*l(1)*lc(2)*sin(Z0(3))*Z0(2)^2                       ];
 
G = [g*cos(Z0(1))*(m(1)*lc(1)+m(2)*l(1))+m(2)*g*lc(2);
          m(2)*g*lc(2)*cos(Z0(1)+Z0(3))              ];
      


%% simulation loop

Mk = double(subs(M,Z0,robot.z0_));
Vk = double(subs(V,Z0,robot.z0_));
Gk = double(subs(G,Z0,robot.z0_));
 