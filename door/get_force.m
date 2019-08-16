%% calculate force on the door
clear all
load('IK_open_th.mat')

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
l1 = 0.4;
lc = 0.4;
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