function [xref_t,xori,xref,xR, A, B ]=gen_loop_MMD(var,z0_,zT,horizon )

nstep = horizon+1;
nstate = size(z0_,1);
%% Arm
% Generate reference
xref_pre =[];
for i = 1:nstate
    xref_pre = [xref_pre; linspace(z0_(i),zT(i),nstep )];
end  

% rearrange reference
xref =[];        
for i=1:nstep
    xref = [xref; xref_pre(:,i)];
end


% Arm xR
xR=[];xR(:,1)=xref(1:nstate);
xori=xref(nstate+1:end);
xref=xori;





% LQR
xref_t = z0_(6)*ones(var.N,1);


% A, B

U0 = zeros(6,1);
[Z1, A, B ] = LinKin(z0_, U0, var.dt);


end