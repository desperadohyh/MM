%% plot distance

clc
clear all
close all
fighandle = [];
fighandle(1) = figure(1); hold on;
set(gcf, 'position', [0 0 500 500]);
fighandle(2) = figure(2); hold on;

    
%% parameter definition
% TB definition
% sampling time
dt          = 0.5;

% TB: trajectory dimension
dim         = 2; %x,y
%%%%%%%%%%%%%%%%%%%%%%%%%%%
gr_push_hold
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Arm parameters
robot=robotproperty_hc(4);
% Arm joint
njoint      =5; % joint number
nstate      =10; % QP variable dim
nu          =5; % acceleration dim 
DH          =robot.DH;

% robot model

tb3.base = [0.065 0.065 -0.195 -0.195;
                0.13  -0.13 -0.13  0.13];
            
tb3.wC = [0    0;
         0.14  -0.14  ;
         0.05 0.05];   % center of circle 
R = 0.03 ;    % Radius of circle 
teta=0:0.5:2*pi ;
for i = 1:2
tb3.wx(i,:) = tb3.wC(1,i)+R*cos(teta);
tb3.wy(i,:) = tb3.wC(2,i)*ones(1,length(teta)) ;
tb3.wz(i,:) = tb3.wC(3,i)+R*sin(teta) ;
end





%%
% Target
target = [-0.2; 0; 0.05];
t_marg = [0.35 0.2 0.35 0; 0  0  0  0; 0 0 0 0];

ss=15;
xV = -0.15;
Vx = xV;

%% Plot


[xref_t,xref,xR,refpath]=generate_reference_loop_hold(var,Ax_current,Tx_current,zAT,zT,horizon,nstate,u0,mode_,target,t_marg );


figure
plot(xref(1:10:end),'-ko')
hold on
plot(xref(2:10:end),'m-d')
hold on
plot(xref(3:10:end),'--x')
plot(xref(4:10:end),'-r*')
plot(xref(5:10:end),'-go')

legend('platform angle','\theta_1','\theta_2','\theta_3','\theta_4' ,'location','eastoutside')
ylabel('Angle[rad]')
xlabel('Time step')

pos={};

color{1} = 'or-';
color{2} = '-kx';
color{3} = 'm-d';
color{4} = '--x';
color{5} = '-yo';

figure
xlabel('x[m]')
ylabel('y[m]')
zlabel('z[m]')
axis equal
for i=1:7:horizon
    % provide mode_ according to current 2D path 
    xy = refpath(i*2+1:(i+1)*2);
    base = [xy' 0.1];
    % get reference theta
    theta=xref(nstate*(i-1)+1:nstate*(i-1)+njoint);
    [end_dis,pos,M]=plot_link(theta,DH(1:njoint,:),base,obs_arm,robot.cap);
    disp('........')
    
    
    tb3_base_w = M{2}(1:2,1:2)*tb3.base+base(1:2)';
    fill3(tb3_base_w(1,:), tb3_base_w(2,:), 0.1*ones(1,4), [0.9-(i/horizon)/1.2,0.9-(i/horizon)/1.2,0.9-(i/horizon)/1.2]);
    fill3(tb3_base_w(1,:), tb3_base_w(2,:), 0.05*ones(1,4), [0.9-(i/horizon)/1.2,0.9-(i/horizon)/1.2,0.9-(i/horizon)/1.2]);
    hold on
    for w = 1:2
    tb3_w_w = M{2}(1:2,1:2)*[tb3.wx(w,:);tb3.wy(w,:)]+base(1:2)';
    fill3(tb3_w_w(1,:), tb3_w_w(2,:), tb3.wz(w,:), [0.9-(i/horizon)/1.2,0.9-(i/horizon)/1.2,0.9-(i/horizon)/1.2]);
    end
    %     ob = Polyhedron('V',[tb3_base_w]');
%     PP =ob.plot('color','b','alpha',((i/horizon))/2);
    
    for j = 1:5
        if j<5
             plot3([ pos{j}.p(1,3), pos{j+1}.p(1,3)],[ pos{j}.p(2,3), pos{j+1}.p(2,3)],[ pos{j}.p(3,3), pos{j+1}.p(3,3)],'k-','color',[1-(i/horizon),1-(i/horizon),1-(i/horizon)/1.5],'LineWidth',3);
        end
       etrj = plot3( pos{j}.p(1,3), pos{j}.p(2,3), pos{j}.p(3,3),color{1},'color',[1-(i/horizon)/3.5,1-(i/horizon)/2.5,1-(i/horizon)],'LineWidth',3);
       
       pos{j}.p(3,3)
       hold on
    end
    xlabel('x[m]')
    ylabel('y[m]')
    zlabel('z[m]')
    axis equal
   % pause
end

axis equal



% plot





