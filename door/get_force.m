%% calculate force on the door/ process signals
clear all
%load('IK_open_th.mat')

load('IK_open1.mat')
dt = 0.2;
ss = size(theta_,2);
theta_ = theta_/180*pi;

theta_(:,end+1) = theta_(:,end);

dee= [];
for j = 1:ss
    dee(:,j) = (theta_(:,j+1)-theta_(:,j))/dt;
end

dee(:,end+1) = dee(:,end);

ddee= [];
for j = 1:ss
    ddee(:,j) = (dee(:,j+1)-dee(:,j))/dt;    
end
ddee(:,end+1) = ddee(:,end);

%% Door simulation

% parameters

m = 1;
l1 = 0.35;
lc = 0.35;
d = 0.01;
Izz = m*((2*l1)^2+d^2)/12;
% K = 4.16*l1;
% B = 6.25*l1;
K = 2.16*l1;
B = 2.25*l1;

M = (m*l1^2 + Izz)/lc;


% get force
F_all = [];

for i = 1:ss
    
    lf = Rgl_f(theta_(i))*pinv(Jk_f(theta_(i)))'*(M*ddee(i)+B*dee(i)+K*theta_(i));

    F_all = [ F_all lf];
end


%% plot

% figure
figure
plot(F_all')
legend('F_{x,l}','F_{y,l}','F_{z,l}')
% plot(z(:,1),z(:,2))


%% process thetas

%theta's

theta_open(:,end+1) = theta_open(:,end);

deet= [];
for j = 1:ss
    deet(:,j) = (theta_open(:,j+1)-theta_open(:,j))/dt;
end

deet(:,end+1) = deet(:,end);

ddeet= [];
for j = 1:ss
    ddeet(:,j) = (deet(:,j+1)-deet(:,j))/dt;    
end
ddeet(:,end+1) = ddeet(:,end);

% traj's
traj = X_out(1:3,:);
traj(:,end+1) = traj(:,end);

deej= [];
for j = 1:ss
    deej(:,j) = (traj(:,j+1)-traj(:,j))/dt;
end

deej(:,end+1) = deej(:,end);

ddeej= [];
for j = 1:ss
    ddeej(:,j) = (deej(:,j+1)-deej(:,j))/dt;    
end
ddeej(:,end+1) = ddeej(:,end);

% end-effector
traj_e = end_effector;
traj_e(:,end+1) = traj_e(:,end);

dee_e= [];
for j = 1:ss
    dee_e(:,j) = (traj_e(:,j+1)-traj_e(:,j))/dt;
end

dee_e(:,end+1) = dee_e(:,end);

ddee_e= [];
for j = 1:ss
    ddee_e(:,j) = (dee_e(:,j+1)-dee_e(:,j))/dt;    
end
ddee_e(:,end+1) = ddee_e(:,end);


Ref.theta_ = theta_;
Ref.dee = dee;
Ref.ddee = ddee;

Ref.theta_open = theta_open;
Ref.deet = deet;
Ref.ddeet = ddeet;

Ref.traj = traj;
Ref.deej = deej;
Ref.ddeej = ddeej;

Ref.traj_e = traj_e;
Ref.dee_e = dee_e;
Ref.ddee_e = ddee_e;


save('IK_open_pidref.mat','Ref','X_out','theta_open','traj_open','end_effector','theta_','X_all')
