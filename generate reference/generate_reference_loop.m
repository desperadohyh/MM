function [xref_t,xori,xref,xR,path,refpath,uref]=generate_reference_loop(var,Ax_current,Tx_current,zAT,horizon,nstate,uref,u0 )

nstep = horizon+1;

%% Arm
% Generate reference
xref_pre = [linspace(Ax_current(1),zAT(1),nstep);
            linspace(Ax_current(2),zAT(2),nstep);
            linspace(Ax_current(3),zAT(3),nstep);
            linspace(Ax_current(4),zAT(4),nstep);
            linspace(Ax_current(5),zAT(5),nstep);
            zeros(5,nstep)];
        

% rearrange reference
xref =[];        
for i=1:nstep
    xref = [xref; xref_pre(:,i)];
end
% Arm xR
xR=[];xR(:,1)=xref(1:nstate);
xori=xref(nstate+1:end);
xref=xori;

%% Turtlebot
% Generate reference
z0(1) = Tx_current(1);
z0(2) = Tx_current(2);
path = [linspace(Tx_current(1),Tx_current(1)+2,nstep);
        linspace(Tx_current(2),0,nstep)];
% rearrange reference    
refpath = [];
for i=1:nstep
    refpath = [refpath;path(:,i)];
end

% LQR

u0 = [u0(:,1:3) zeros(2,var.N-4)];

xref_t = Tx_current(3)*ones(var.N,1);

end