close all 
clear all

fighandle = [];
fighandle(1) = figure(1); hold on;
fighandle(2) = figure(2); hold on;

z0 = [0;0];
zT = [2;0];

horizon = 24;
nstep = horizon+1;

path = [linspace(z0(1),zT(1),nstep);
        linspace(z0(2),zT(2),nstep)];
    

% sampling time
dt = 0.5;
% number of obstacles
nobj = 1;
obs = {};
obs{1}.v = [0;0.1*dt];
obs{1}.poly = [0.8 1.3 1.4 0.7;0.1 0.1 -0.5 -0.5];




nstep = size(path,2);
dim = 2; %x,y 

% 1= consider input(angular velocity), 0= not considering input 
SCFS = 1;
%% The cost function% The weight
    c = [1,10,20];
    % The distance metric between the original path and the new path
    Q1 = eye(nstep*dim);
    %Q1((nstep-1)*dim+1:end,(nstep-1)*dim+1:end) =  eye(dim)*1000;
    %Q1(1:dim,1:dim) =  eye(dim)*1000;
    % The velocity
    Vdiff = eye(nstep*dim)-diag(ones(1,(nstep-1)*dim),dim);
    Vconst = [-eye(2) eye(2) zeros(2,(nstep-2)*2);[[zeros((nstep-1)*2,2) eye((nstep-1)*2) ]-[eye((nstep-1)*2) zeros((nstep-1)*2,2)]]];
    V_ratio = [5 0;0 1];
    Rpenalty = kron(eye(nstep),V_ratio);
    Q2 = Vconst'*Rpenalty'*Rpenalty*Vconst;
    %Q2 = Vdiff(1:(nstep-1)*dim,:)'*Q1(1+dim:end,1+dim:end)*Vdiff(1:(nstep-1)*dim,:);
    Vref = [2,0]*dt;    
    Vref_1 = c(2)*kron(ones(1, nstep),Vref)*Rpenalty'*Rpenalty*Vconst;
    % The accelaration
    Vdiff = eye(nstep*dim)-diag(ones(1,(nstep-1)*dim),dim);
    Adiff = Vdiff-diag(ones(1,(nstep-1)*dim),dim)+diag(ones(1,(nstep-2)*dim),dim*2);
    Q3 = Adiff(1:(nstep-2)*dim,:)'*Adiff(1:(nstep-2)*dim,:);   
%% Cost function update
    dir = [zT(1)-(-6) zT(2)-0];
    dir_T = (1/norm(dir))*[ zT(2)-0 -zT(1)+(-6)];
    dd = kron(ones(nstep,1),dir_T*[-6;0]);
    D = kron(eye(nstep),dir_T);
    % Distance to reference line
    Q1 = D'*D;
    Xdis_1 = 2*c(1)*dd'*D;
    % The total costj
    Qref = 1*(Q1*c(1)+Q2*c(2)+Q3*c(3));
    Qabs = 0*Q3*c(3);
%% Extended cost
Mcurv = eye(nstep);
Mcurv(nstep,nstep) = 5;
Vcurv = eye(nstep)-diag(ones(1,nstep-1),1);
Acurv = Vcurv-diag(ones(1,(nstep-1)),1)+diag(ones(1,(nstep-2)),2);
Qcurv = 5*Mcurv;%+Vcurv(1:nstep-1,:)'*Vcurv(1:nstep-1,:)+Acurv(1:(nstep-2),:)'*Acurv(1:(nstep-2),:);
%% The boundary constraint
Aeq = zeros(4*dim,nstep*dim+nstep);
Aeq(0*dim+1:1*dim,1:dim) = eye(dim);
Aeq(1*dim+1:2*dim,(nstep-1)*dim+1:nstep*dim) = eye(dim);
Aeq(2*dim+1:3*dim,1:2*dim) = [-eye(dim) eye(dim)];
Aeq(3*dim+1:4*dim,(nstep-2)*dim+1:nstep*dim) = [-eye(dim) eye(dim)];
beq = [path(:,1);path(:,end);path(:,2)-path(:,1);path(:,end)-path(:,end-1)];
%% The Iteration
refpath = [];
for i=1:nstep
    refpath = [refpath;path(:,i)];
end
oripath = refpath;
refinput = ones(1,nstep);
P = [0 1;-1 0];
tic
for k = 1:10
FEAS = 0;
%% The constraint
Lstack = []; Sstack = []; margin = 0.1;
for i=1:nstep
    for j=1:nobj
        poly = obs{j}.poly+obs{j}.v*ones(1,4)*dt*i;
        [L,S,d] = d2poly(refpath((i-1)*dim+1:i*dim)',poly');
        Lstack = [Lstack;zeros(1,(i-1)*dim) L zeros(1,(nstep-i)*dim) zeros(1,nstep)];
        Sstack = [Sstack;S-margin];
    end
    if FEAS > 0 && SCFS > 0
    if i>2
        xk0 = refpath((i-1)*dim+1:(i-0)*dim);
        xk1 = refpath((i-2)*dim+1:(i-1)*dim);
        xk2 = refpath((i-3)*dim+1:(i-2)*dim);
        ur = refinput(i);
        ltheta = norm(xk0-xk1)^2;
        lk0 = 2*ur*(xk0-xk1)'+(xk1-xk2)'*P';
        lk1 = -2*ur*(xk0-xk1)'-(xk1-xk2)'*P'+(xk0-xk1)'*P;
        lk2 = -(xk0-xk1)'*P;
        s = norm(xk0-xk1)^2*ur+(xk0-xk1)'*P*(xk1-xk2)-ltheta*ur-lk0*xk0-lk1*xk1-lk2*xk2;
        Lstack = [Lstack;zeros(1,(i-3)*dim) -lk2 -lk1 -lk0 zeros(1,(nstep-i)*dim) zeros(1,i-1) -ltheta zeros(1,nstep-i)];
        Sstack = [Sstack;s];
        ltheta = norm(xk0-xk1)^2;
        lk0 = 2*ur*(xk0-xk1)'-(xk1-xk2)'*P';
        lk1 = -2*ur*(xk0-xk1)'+(xk1-xk2)'*P'-(xk0-xk1)'*P;
        lk2 = (xk0-xk1)'*P;
        s = norm(xk0-xk1)^2*ur-(xk0-xk1)'*P*(xk1-xk2)-ltheta*ur-lk0*xk0-lk1*xk1-lk2*xk2;
        Lstack = [Lstack;zeros(1,(i-3)*dim) -lk2 -lk1 -lk0 zeros(1,(nstep-i)*dim) zeros(1,i-1) -ltheta zeros(1,nstep-i)];
        Sstack = [Sstack;s];
    end
    end
end

%% QP
if FEAS > 0 && SCFS > 0
    Qe = blkdiag(Qref+Qabs,1*Qcurv);
else
    Qe = blkdiag(Qref+Qabs,0*Qcurv);
end
soln = quadprog(Qe,[-Qref*oripath;zeros(nstep,1)],Lstack,Sstack,Aeq,beq);
pathnew = soln(1:dim*nstep);
refinput = soln(dim*nstep+1:end);


figure(fighandle(1));
plot(pathnew(1:dim:end),pathnew(2:dim:end),'-*','color',[1-k/6,1-k/6,1-k/6])
figure(fighandle(2));
plot(refinput,'color',[1-k/6,1-k/6,1-k/6]);
if norm(refpath-pathnew) < 0.1
    disp(['converged at step ',num2str(k)]);
    break
end
refpath = pathnew;
end
%%
time = toc
disp(['final cost: ']);
cost_curv(pathnew,oripath,Qref,Qabs,Qcurv,nstep)

%%
figure(fighandle(1));
plot(pathnew(1:dim:end),pathnew(2:dim:end))
%plot(path(1,:),path(2,:),'b')
plot(pathnew(1:dim:end),pathnew(2:dim:end),'r')
ob = Polyhedron('V',obs{1}.poly');
ob.plot('color','g');
axis equal
grid off
box on
legend('Iter1','Iter2','Iter3','Iter4','Iter5')
% 
% figure(fighandle(2));
% legend('Iter1','Iter2','Iter3','Iter4','Iter5')
% figure;clf; hold on
% curv = [];
% for i=3:nstep
%     l = (refpath((i-2)*dim+1:(i-1)*dim)-refpath((i-3)*dim+1:(i-2)*dim))'*P;
%     s = l*(2*refpath((i-2)*dim+1:(i-1)*dim)-refpath((i-3)*dim+1:(i-2)*dim));
%     v = norm(refpath((i-2)*dim+1:(i-1)*dim)-refpath((i-3)*dim+1:(i-2)*dim))^2;
%     curv(i) = (l*refpath((i-1)*dim+1:i*dim) - s)/v;
% end
% plot(curv);