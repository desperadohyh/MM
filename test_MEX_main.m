% test MEX


a=3;
b=4;
gg = cell{1,100};
for cc = 1:100
gg{1,cc} = testMEX(a,b);
end