clearglobal
clear
clc
close();
rand("seed",25);
p = rand(500,2)*10;
getd(".");
getd("./kNN");
global cellIn;
global lcell;
cellIn=cell();
lcell=0;

tic();
kd_tree(p,0,0);
disp('kdtree created in ' + string(toc()) + ' seconds');

pt = 25;

nNeigh = [1 2 5 10 30 50 100 200];
time_kd = [];
time_bf = [];

for i = 1:length(nNeigh)
    tic();
    n1 = kNN(cellIn,nNeigh(i),p(pt,:));
    t_kd = toc();
    time_kd = [time_kd;t_kd];
    tic();
    n2 = bruteForceNeighbors(p,p(pt,:),nNeigh(i));
    t_bf = toc();
    time_bf = [time_bf;t_bf];
end:
