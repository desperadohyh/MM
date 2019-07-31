function plot_arm(ss,theta_implement,robot,gap,flip)
pos={};

links_=[];
etrj=[];

points = size(theta_implement,1)+1;

T = robot.T;
%% Plot

for i=2:gap:ss
%     delete(plotbase2)
%     delete(plotbase1)
%     delete(plotwheels)
%     delete(links_)
%     delete(etrj)
%     delete(plotdoor)
    % get bace position    
    
    % get reference theta
    theta=theta_implement(:,i);
    %[pos,M]=plot_link_s(theta,robot.base',robot.cap,flip);
        
    [pos,M]=plot_link_4(T,theta,robot.base',robot.cap,flip);
    
    
    
    
    % plot arm
    for j = 1:points
        if j<points
       links_(j) = plot3([ pos{j}.p(1,3), pos{j+1}.p(1,3)],[ pos{j}.p(2,3), pos{j+1}.p(2,3)],[ pos{j}.p(3,3), pos{j+1}.p(3,3)],'k-','color',[1-(i/ss),1-(i/ss),1-(i/ss)/1.5],'LineWidth',3);
        end
       etrj(j) = plot3( pos{j}.p(1,3), pos{j}.p(2,3), pos{j}.p(3,3),'o-','color',[1-(i/ss)/3.5,1-(i/ss)/2.5,1-(i/ss)],'LineWidth',3);
       hold on
    end
    
   
end

xlabel('x[m]')
ylabel('y[m]')
zlabel('z[m]')

axis equal

end