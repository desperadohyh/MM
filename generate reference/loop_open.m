function [xori,xref,xref_pre,xR, A, B ]=loop_open(var,z0_,zT,horizon,zTt,Refss )

nstep = horizon+1;
nstate = size(z0_,1);
Ts = var.dt;
H = var.H;
Vref = var.Vref;
r = var.r;

%% Arm
% Generate reference
xref_pre =[];
ang = Ts*H*norm(Vref)/r;
ang_v = norm(Vref)/r;
zT = [z0_(1)+Ts*H*Vref(1) 0 Vref' norm(Vref)  0 0 z0_(8)+ang ang_v z0_(10)+ang  ang_v  Refss(end-8:end)' ];

if zT(1)>zTt(1)
    zT(1)=zTt(1);   
end

    delay = 2;
for i = 1:nstate
    xref_pre = [xref_pre; [linspace(z0_(i),zT(i),delay) zT(i)*ones(1,nstep-delay)]];
end  

% rearrange reference
xref =[];        
for i=1:nstep
    xref = [xref; xref_pre(:,i)];
end


% Arm xR
xR=[];xR(:,1)=xref(1:nstate);
xori=xref(nstate+1:end);
%xref=xori;





% LQR
xref_t = z0_(6)*ones(var.N,1);


% A, B

U0 = zeros(6,1);
[Z1, A, B ] = LinKin(z0_, U0, var.dt);

% update base lacation

 robot.base = [z0_(1) z0_(2) 0.1]';


end