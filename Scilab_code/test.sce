clearglobal
clear
clc
close();
rand("seed",25);
p = rand(100,2)*10;
getd(".")
global cellIn;
global lcell;
cellIn=cell();
lcell=0;
kd_tree(p,0,0);
pt = 1;
nNeigh = 1;

n1 = kNN(cellIn,nNeigh,p(pt,:));

n2 = bruteForceNeighbors(p,p(pt,:),nNeigh);

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

