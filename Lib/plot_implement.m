function plot_implement(ss,theta_implement,traj_implement,robot,tb3,gap)
pos={};


%% Plot

for i=1:gap:ss
    % get bace position    
    base = [traj_implement(:,i); 0.1];
    % get reference theta
    theta=theta_implement(:,i);
    [pos,M]=plot_link(theta,base',robot.cap);
        
    % plot base
    tb3_base_w = M{2}(1:2,1:2)*tb3.base+base(1:2);
    fill3(tb3_base_w(1,:), tb3_base_w(2,:), 0.1*ones(1,4), ...
        [0.9-(i/ss)/1.2,0.9-(i/ss)/1.2,0.9-(i/ss)/1.2]);
    hold on
    fill3(tb3_base_w(1,:), tb3_base_w(2,:), 0.05*ones(1,4),...
        [0.9-(i/ss)/1.2,0.9-(i/ss)/1.2,0.9-(i/ss)/1.2]);
    
    
    % plot wheels
    for w = 1:2
    tb3_w_w = M{2}(1:2,1:2)*[tb3.wx(w,:);tb3.wy(w,:)]+base(1:2);
    fill3(tb3_w_w(1,:), tb3_w_w(2,:), tb3.wz(w,:),...
        [0.9-(i/ss)/1.2,0.9-(i/ss)/1.2,0.9-(i/ss)/1.2]);
    end
    
    % plot arm
    for j = 1:6
        if j<6
             plot3([ pos{j}.p(1,3), pos{j+1}.p(1,3)],[ pos{j}.p(2,3), pos{j+1}.p(2,3)],[ pos{j}.p(3,3), pos{j+1}.p(3,3)],'k-','color',[1-(i/ss),1-(i/ss),1-(i/ss)/1.5],'LineWidth',3);
        end
       etrj = plot3( pos{j}.p(1,3), pos{j}.p(2,3), pos{j}.p(3,3),'o-','color',[1-(i/ss)/3.5,1-(i/ss)/2.5,1-(i/ss)],'LineWidth',3);
       hold on
    end   
    pause
end

xlabel('x[m]')
ylabel('y[m]')
zlabel('z[m]')
axis equal

end