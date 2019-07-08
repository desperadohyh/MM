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
%%
SW_PAA_ON         = 1;
%%

% parameters
m = 1;
l1 = 0.3;
lc = 0.3;
d = 0.01;
Izz = m*((2*l1)^2+d^2)/12;
K = 4.16*l1;
B = 6.25*l1;

M = (m*l1^2 + Izz)/lc;
F = 1.9;

% continuous time model
h = tf(1,[M B K]);
% discrete time model
hd = c2d(h,0.05);
% get dtf parameters
[num,den,Ts] = tfdata(hd)
theta.signals.values(1,:)=[den{1}(2:3) num{1}(2:3)];
count = 1;
y.time=1.5;
y.signals.values=1;
y.signals.dimensions=1;
load('save_th.mat')
%%


disp('SELECT TEST INPUT SIGNAL OPTIONS:')
disp('0 (default) ---- step input signal')
disp('1           ---- sinusoidal signal with a single frequency')
disp('2           ---- sinusoidal signal with two frequencies')
disp('3           ---- white nosie')
disp('   ')
disp('Press ENTER for default selection.')
FLAG_INPUT_SIGNAL = input(' ');
if isempty(FLAG_INPUT_SIGNAL)
    FLAG_INPUT_SIGNAL = 0;
end
disp('SELECT PAA ALGORITHM TYPE:')
disp('0 (default) ---- series-parallel')
disp('1           ---- parallel')
disp('2           ---- two-stage: series-parallel first and then parallel')
disp('Press ENTER for default selection.')
FLAG_ALG = input(' ');
if isempty(FLAG_ALG)
    FLAG_ALG = 1;
end
disp('SELECT TO TURN ON NOISE OR NOT:')
disp('0 (default) ---- noise off')
disp('1           ---- noise on')
disp('Press ENTER for default selection.')
FLAG_NOISE_ON = input(' ');
if isempty(FLAG_NOISE_ON)
    FLAG_NOISE_ON = 0;
end
disp('Do you want to use forgetting factor?')
disp('No (default)')
disp('Yes')
disp('Press ENTER for default selection.')
FLAG_FORGETTING_FACTOR = input(' ');
if isempty(FLAG_FORGETTING_FACTOR)
    FLAG_FORGETTING_FACTOR = 0;
end
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
%% configure the actual plant infor
plant_denominator_spr_parallel = [1, -0.1, 0.64];
plant_denominator_nspr_parallel = [1, -1.6, 0.64];
%%
simuName    = 'DOOR';
%% run the algorithms
Ts          = 0.01; % you are free to set the sampling time as you wish
t_sim       = 4; % total simulation time. Again, set it as you wish.
t_AdapOn    = 0; % set the time when the adaptation algs are turned on
t_AdapOff   = t_sim+1; % set the time when the adaptation algs are turned off

sim(simuName);
figure, plot(yhat.time, y.signals.values - yhat.signals.values);
title('posterior error'); grid on;
figure, plot(theta.time, theta.signals.values); title('evolution of \theta');
legend('a1', 'a2', 'b0', 'b1'); grid on;
if FLAG_FORGETTING_FACTOR
    figure, plot(lambda.time, lambda.signals.values); 
    title('forgetting factor \lambda');
end
