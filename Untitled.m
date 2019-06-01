
close all

plot3(1 ,2, 3,'r*')
hold on
% A=[-sqrt(6)/2,-3*sqrt(2)/2,-sqrt(3)/2;sqrt(6),0,-sqrt(3)/2;-sqrt(6)/2,3*sqrt(2)/2,-sqrt(3)/2;0,0,1]
% b=[-sqrt(6)/2;-sqrt(6)/2;-sqrt(6)/2;0];
A =[1 0 0;
    -1 0 0;
    0 1 0;
    0 -1 0 
    ];
b = [0 ;-1;0 ;-1];
bu= [5;5 ;0];
bl = [-1;0;0];

plotregion(A,b,bl,bu,[0.1,0.5,0.8]);
xlabel('x')

axis equal

%%
close all
plotregion([1 1],[0.6],[0 0],[1 1],'m',0.3,4*(rand(2,7)-0.5),'r:','gxbx');
axis equal