clearglobal
clear
clc
close();
rand("seed",10);
p = rand(50,2)*10;
getd(".")
global cellIn;
global lcell;
cellIn=cell();
lcell=0;
kd_tree(p,0,0);
pt = 5;
n1 = kNN(cellIn,3,p(pt,:));

n2 = bruteForceNeighbors(p,p(pt,:),3);

//plot(p(:,1),p(:,2));
scatter(p(:,1),p(:,2));
scatter([p(pt,1),p(pt,1)],[p(pt,2),p(pt,2)],36,'red');
plot(n2(:,1),n2(:,2),n1(:,1),n1(:,2));
