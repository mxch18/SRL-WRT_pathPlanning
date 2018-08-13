clearglobal
clear
clc
close();
rand("seed",25);
p = rand(1000,2)*10;
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
nNeigh = 5;

tic();
n1 = kNN(cellIn,nNeigh,p(pt,:));
disp('Time for kNN search in kdtree:'+ string(toc()));

tic();
n2 = bruteForceNeighbors(p,p(pt,:),nNeigh);
disp('Time for kNN search with naive search:'+ string(toc()));

if size(p,2)==2 then
    scatter(p(:,1),p(:,2));
    scatter([p(pt,1),p(pt,1)],[p(pt,2),p(pt,2)],36,'red');
    plotPts(n1,'og')
    plotPts(n2,'oy')
elseif size(p,2)==3 then
    scatter3(p(:,1),p(:,2),p(:,3));
    scatter3([p(pt,1),p(pt,1),p(pt,1)],[p(pt,2),p(pt,2),p(pt,2)],[p(pt,3),p(pt,3),p(pt,3)],36,'red');
//    plot3d3(n2(:,1),n2(:,2),n2(:,3),flag=[0,0,2,4])
//    plot3d2(n1(:,1),n1(:,2),n1(:,3),flag=[5,5,2,4])
    plotPts(n2,4,8,5,5)
    plotPts(n1,4,8,0,0)
end

