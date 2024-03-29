% % parameter identification algorithms
% % This template is for the parameter estimation of second order systems
% in the following form:
%                 b0+b1*z^-1
% y(k) = ----------------------u(k-1)
%            1+a1*z^-1 +a2*z^-2
% ==> theta = [a1, a2, b0, b1]', phi = [-y(k-1), -y(k-2), u(k-1), u(k-2)]';
% Feel free to extend it to higher order systems as you like.
% ============================================================
%   Copyright (c) 2015-, Liting Sun, litingsun@berkeley.edu
%   Author(s): Liting Sun
%              University of California at Berkeley
%              Berkeley, CA, 94720, USA
% ============================================================
%%
clear all;
close all;
fighandle(1) = figure(1); hold on;
%%
SW_PAA_ON         = 1;

%%
FLAG_INPUT_SIGNAL = 0;
FLAG_ALG = 1;
FLAG_NOISE_ON = 0;
FLAG_FORGETTING_FACTOR = 0;

%% initialize the parameters for PAA
theta_init  = [0.7089, 0.3539, 3.0, 1.0217]';
F_init      = 1e+10*eye(4); % you may need to tune this for different algorithms
adap_init.F             = F_init;
adap_init.theta         = theta_init;
adap_init.Alg           = FLAG_ALG;
if FLAG_NOISE_ON
    % for different input (excitation) signals, you may need to tune the
    % gain of the noise
    noise_gain          = 0.1;
else
    noise_gain          = 0;
end
if FLAG_FORGETTING_FACTOR
    % for exponentially increasing forgetting factor
    adap_init.lambda_init   = 0.98;
    adap_init.lambda_end    = 1.0;
else
    adap_init.lambda_init   = 1.0;
    adap_init.lambda_end    = 1.0;
end

%% Parameter setup
simuName    = 'DOOR_prob_done';
Ts          = 0.01; % you are free to set the sampling time as you wish
t_sim       = 4; % total simulation time. Again, set it as you wish.
t_AdapOn    = 0; % set the time when the adaptation algs are turned on
t_AdapOff   = t_sim+1; % set the time when the adaptation algs are turned off

% pick related
% parameters
g = 9.81;
r=0.01;
m1 = 0.5;
l1 = 0.2;

m2 = 0.2;
l2 = 0.2;

ld = 0.05;
lc = 0.05; %othe object covering
lf = 0.06; %gap
lp = 0.00;

[I, Tau] = IG_update(m1,l1,m2,l2,ld,lp,r,g);

K = 0.4;
B = 0.06;

M = I;
F = Tau;

% process model
Init_ck = [0.9 0.1];
Gpm = [0.9 0.1;
    0.05 0.95]; 

% continuous time model
h = tf(1,[M B K]);
% discrete time model
hd = c2d(h,0.05);
% get dtf parameters
[num,den,Ts] = tfdata(hd);
theta.signals.values(1,:)=[den{1}(2:3) num{1}(2:3)];
count = 1;
y.time=1.5;
y.signals.values=1;
y.signals.dimensions=1;
load('save_th.mat')

%% run the algorithms
% initialize 
pick_ok = false;
lp_all = [0];
b1 = [0 0 1]';
c_k = 1;
count = 1;

sim(simuName);
dryi = mean(y.signals.values);
while pick_ok == false
    [I, Tau] = IG_update(m1,l1,m2,l2,ld,lp,r,g);
    [num,den,Ts,theta,F, M] = model_update(I, B, K,Tau);
    sim(simuName);
    fighandle(1); plot(y.time, y.signals.values); title('output angle');
    y_all = y.signals.values;
    dry = mean(y_all);
    error = norm(y_all - mean(y_all));
    
    % check tile and adjust
    if error<0.05
        pick_ok = true;
        b(1) = 1;
        b(2) = 0;
        b(3) = 0;
        
    else        
        % b(c_k)
        if sign(dryi)<0
            b(1) = 1;
            b(2) = 0;
            b(3) = 0;
        elseif lp == (l1/2-lc)
            error = 1;
            b(1) = 0;
            if y_all(end)*(l1/2)+r>=lf/2
                b(2)=0;
                b(3)=1;
            else
                b(2)=1;
                b(3)=0;
            end            
        else
            b(1) = exp(-norm(dry/((l1/2-lc))));
            gap = y_all(end)*(l1/2)+r-lf/2
            if gap>0
                b2 = exp(-norm(gap/(l1/2-lc)));
                b3 = exp(-norm((l1/2-lc)/(gap*50)))
                b(2) = (1-b(1))*(b2/(b2+b3));
                b(3) = (1-b(1))*(b3/(b2+b3))
                %pause
            else
                b(2) = (1-b(1));
                b(3) = 0;
            end
        end
        
        % update lp
        if dry*dryi<0            
            lp = lp/2;
        else            
            lp = lp*(1+dry*0.1)+error*0.02*sign(dry);%*(1/count) ; % 0.06
            if lp>lc
                lp=lc;
            end            
        end
        lp_all = [lp_all lp];
        
    end
    b1 = [b1 b'];
    count = count +1;
    %pause
end
%% plot
figure; plot(lp_all','-b');
xlabel('Trials')
ylabel('Distance from the geometric center to the grasping point [m]')
figure; bar(b1','DisplayName','b1');
legend('b(c_k=1)','b(c_k=2)','b(c_k=3)','Location','eastoutside')
xlabel('Trials')
ylabel('b(c_k )')
% others
% figure, plot(yhat.time, y.signals.values - yhat.signals.values,'r*');
% title('posterior error'); grid on;
% figure, plot(theta.time, theta.signals.values); title('evolution of \theta');
% legend('a1', 'a2', 'b0', 'b1'); grid on;
% if FLAG_FORGETTING_FACTOR
%     figure, plot(lambda.time, lambda.signals.values); 
%     title('forgetting factor \lambda');
% end
% %%
% figure, plot(theta.time, b_c.signals.values); %title('evolution of p(c_k|z_k,s^\hat_k');
% axis([0 5 -0.5 1.5]); grid on;
% xlabel('Time [s]')
% legend('b(ck =1)','b(ck =2)')
% 
% figure, plot(y.time, y.signals.values); title('output angle');
% axis([0 5 -0.1 2]); grid on;
% ylabel('Angle [rad]')
% xlabel('Time [s]')

