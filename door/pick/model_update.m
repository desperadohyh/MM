function [num,den,Ts,theta,F, M] = model_update(I, B, K,Tau)

M = I;
F = Tau;

% continuous time model
h = tf(1,[M B K]);
% discrete time model
hd = c2d(h,0.05);
% get dtf parameters
[num,den,Ts] = tfdata(hd);
theta.signals.values(1,:)=[den{1}(2:3) num{1}(2:3)];
end