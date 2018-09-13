// kd-tree benchmark
clearglobal
clear
clc
close();
getd(".");
getd("./kNN");
rand("seed",25);

nb_pts = [1 10 100 300 500 1000];

global cellIn;
global lcell;

cellIn=cell();
lcell=0;

times = [];


for i = 1:length(nb_pts)
    p = rand(nb_pts(i),2)*10;
    mprintf("Building kd-tree with %d points\n",nb_pts(i));
    cellIn=cell();
    lcell=0;
    tic();
    kd_tree(p,0,0);
    t = toc();
    times = [times; t];
end

plot2d("ln",nb_pts,times);
