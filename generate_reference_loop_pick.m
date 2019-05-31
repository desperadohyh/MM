function [xref_t,xref,xR,refpath]=generate_reference_loop_pick(var,Ax_current,Tx_current,zAT,horizon,nstate,u0,mode,target,t_marg )

nstep = horizon+1;

%% Mode generate reference

switch mode
    case 1  %approach
        % Arm
        
        delay = 2;        
        xref_pre = [[linspace(Ax_current(1),zAT(1),delay) zAT(1)*ones(1,nstep-delay)];
                    [linspace(Ax_current(2),zAT(2),delay) zAT(2)*ones(1,nstep-delay)];
                    [linspace(Ax_current(3),zAT(3),delay) zAT(3)*ones(1,nstep-delay)];
                    [linspace(Ax_current(4),zAT(4),delay) zAT(4)*ones(1,nstep-delay)];
                    [linspace(Ax_current(5),zAT(5),delay) zAT(5)*ones(1,nstep-delay)];
                    zeros(5,nstep)];
                
        % Turtlebot
        
        
        path = [linspace(Tx_current(1),Tx_current(1)-2,nstep);
                linspace(Tx_current(2),0,nstep)];
        
    case 2  % Retry
         delay = 2;        
        xref_pre = [[linspace(Ax_current(1),zAT(1),delay) zAT(1)*ones(1,nstep-delay)];
                    [linspace(Ax_current(2),zAT(2),delay) zAT(2)*ones(1,nstep-delay)];
                    [linspace(Ax_current(3),zAT(3),delay) zAT(3)*ones(1,nstep-delay)];
                    [linspace(Ax_current(4),zAT(4),delay) zAT(4)*ones(1,nstep-delay)];
                    [linspace(Ax_current(5),zAT(5),delay) zAT(5)*ones(1,nstep-delay)];
                    zeros(5,nstep)];
                
        % Turtlebot
        
        
        path = [linspace(Tx_current(1),Tx_current(1)-2,nstep);
                linspace(Tx_current(2),0,nstep)];

    case 3  % Leave
        
        % Arm
        delay = 2;        
        xref_pre = [[linspace(Ax_current(1),zAT(1),delay) zAT(1)*ones(1,nstep-delay)];
                    [linspace(Ax_current(2),zAT(2),delay) zAT(2)*ones(1,nstep-delay)];
                    [linspace(Ax_current(3),zAT(3),delay) zAT(3)*ones(1,nstep-delay)];
                    [linspace(Ax_current(4),zAT(4),delay) zAT(4)*ones(1,nstep-delay)];
                    [linspace(Ax_current(5),zAT(5),delay) zAT(5)*ones(1,nstep-delay)];
                    zeros(5,nstep)];
                
        % Turtlebot
        
        path = [linspace(Tx_current(1),Tx_current(1)+2,nstep);
                linspace(Tx_current(2),0,nstep)];
            
    case 4  % Go back to neutral
        % Arm
        
        delay = 2;        
        xref_pre = [[linspace(Ax_current(1),zAT(1),delay) zAT(1)*ones(1,nstep-delay)];
                    [linspace(Ax_current(2),zAT(2),delay) zAT(2)*ones(1,nstep-delay)];
                    [linspace(Ax_current(3),zAT(3),delay) zAT(3)*ones(1,nstep-delay)];
                    [linspace(Ax_current(4),zAT(4),delay) zAT(4)*ones(1,nstep-delay)];
                    [linspace(Ax_current(5),zAT(5),delay) zAT(5)*ones(1,nstep-delay)];
                    zeros(5,nstep)];
                
        % Turtlebot
        
        
        path = [linspace(Tx_current(1),Tx_current(1)-2,nstep);
                linspace(Tx_current(2),0,nstep)];   
            
       
        
        
           
end

        
%% Rearrange reference   
% Arm
xref =[];        
for i=1:nstep
    xref = [xref; xref_pre(:,i)];
end
% Arm xR (z0_)
xR=[];xR(:,1)=xref(1:nstate);
xori=xref(nstate+1:end);
xref=xori;

% Turtlebot 
refpath = [];
for i=1:nstep
    refpath = [refpath;path(:,i)];
end

%% LQR
u0 = [u0(:,1:3) zeros(2,var.N-4)];
xref_t = Tx_current(3)*ones(var.N,1);


end