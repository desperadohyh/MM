function [end_all] = plot_MM5(ss,theta_implement,traj_implement,robot,tb3,gap,door,v,fighandle)
pos={};
plotbase2=[];
plotbase1=[];
plotwheels=[];
links_=[];
etrj=[];
plotdoor=[];
end_all = [];

%% Plot

for i=1:gap:ss
    
    delete(plotbase2)
    delete(plotbase1)
    delete(plotwheels)
    delete(links_)
    delete(etrj)
%     delete(plotdoor)
    % get bace position    
    base = [traj_implement(:,i); 0.1];
    % get reference theta
    theta=theta_implement(:,i);
    [pos,M]=plot_MMlink(theta,base',robot.cap);
        
    % plot base
    tb3_base_w = M{2}(1:2,1:2)*tb3.base+base(1:2);
    plotbase2 =fill3(tb3_base_w(1,:), tb3_base_w(2,:), 0.1*ones(1,4), ...
        [0.9-(i/ss)/1.2,0.9-(i/ss)/1.2,0.9-(i/ss)/1.2]);
    hold on
    plotbase1 = fill3(tb3_base_w(1,:), tb3_base_w(2,:), 0.05*ones(1,4),...
        [0.9-(i/ss)/1.2,0.9-(i/ss)/1.2,0.9-(i/ss)/1.2]);
    
    
    % plot wheels
    for w = 1:2
    tb3_w_w = M{2}(1:2,1:2)*[tb3.wx(w,:);tb3.wy(w,:)]+base(1:2);
    plotwheels(w) = fill3(tb3_w_w(1,:), tb3_w_w(2,:), tb3.wz(w,:),...
        [0.9-(i/ss)/1.2,0.9-(i/ss)/1.2,0.9-(i/ss)/1.2]);
    end
    
    % plot arm
    for j = 1:6
        if j<6
       links_(j) = plot3([ pos{j}.p(1,3), pos{j+1}.p(1,3)],[ pos{j}.p(2,3), pos{j+1}.p(2,3)],[ pos{j}.p(3,3), pos{j+1}.p(3,3)],'k-','color',[1-(i/ss),1-(i/ss),1-(i/ss)/1.5],'LineWidth',3);
        end
       
       etrj(j) = plot3( pos{j}.p(1,3), pos{j}.p(2,3), pos{j}.p(3,3),'o-','color',[1-(i/ss)/3.5,1-(i/ss)/2.5,1-(i/ss)],'LineWidth',3);
       hold on
    end
    end_all = [end_all pos{j}.p(1:3,3)];    
    xlabel('x[m]')
    ylabel('y[m]')
    zlabel('z[m]')
    axis equal
    axis([-0.5 1 -1.5 0.5 0 0.4])
    pause(0.05)
    if ~isempty(v)
        frame = getframe(fighandle(1));
        writeVideo(v,frame);
    end
    % plot door
    if door ==0
        continue
    else
    c = [0;0.35];
    
    line = (pos{6}.p(1:2,3)-c)*0.8/norm((pos{6}.p(1:2,3)-c));
    door = [line line zeros(2,2); 0.4 0 0 0.4]+[0;0.35;0];
    plotdoor = fill3(door(1,:), door(2,:), door(3,:), ...
        [0.9-(i/ss)/1.2,0.9-(i/ss)/1.2,0.9-(i/ss)/1.2]);
    xlabel('x[m]')
    ylabel('y[m]')
    zlabel('z[m]')
    axis equal
    axis([-0.5 1 -1.5 0.5 0 0.4])
    %axis([-0.5 0.7 -0.3 0.5 0 0.4])
    %pause
    end
    pause%(0.05)
end

xlabel('x[m]')
ylabel('y[m]')
zlabel('z[m]')

axis equal

end