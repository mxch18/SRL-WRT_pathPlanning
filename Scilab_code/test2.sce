clear;close();clc;getd(".");getd("./kNN");getd("./RLG");getd("./RLG/Geometry");getd("./Pruning_strategy")

orig = floor(rand(1,3)*10);
//orig = [1,1,1];
n = floor(rand(1,3)*10);
//n = [1,1,1];
[x,y,z] = rect3D(orig,n);

//xtoproj = floor(rand(1,3)*10);
xtoproj = [1 0 0];
xproj = projectionPlan(xtoproj,orig,n);

//disp(cross(n,xproj-xtoproj))

//scatter3(xtoproj(1)*ones(1,2),xtoproj(2)*ones(1,2),xtoproj(3)*ones(1,2),50,'red');
//scatter3(xproj(1)*ones(1,2),xproj(2)*ones(1,2),xproj(3)*ones(1,2),50,'green');

alp=linspace(-%pi/2,%pi/2,10);
tet=linspace(0,%pi,10);
rHS = 5;
dirHS = rand(1,3);
ctLift = floor(rand(1,3)*50);
[xHS,yHS,zHS]=halfSph(ctLift,rHS,alp,tet,dirHS);
[xHS1,yHS1,zHS1]=halfSph(ctLift+[1 1 0]*2,rHS,alp,tet,dirHS+rand(1,3));
[xHS2,yHS2,zHS2]=halfSph(ctLift+[0 -1 -1]*2,rHS,alp,tet,dirHS+rand(1,3));
[xHS3,yHS3,zHS3]=halfSph(ctLift+[-1 0 1]*2,rHS,alp,tet,dirHS+rand(1,3));

for i = 1:length(xHS)
    v = projectionPlan([xHS(i),yHS(i),zHS(i)],orig,n);
    xHS(i) = v(1);yHS(i) = v(2);zHS(i) = v(3);
    
    v = projectionPlan([xHS1(i),yHS1(i),zHS1(i)],orig,n);
    xHS1(i) = v(1);yHS1(i) = v(2);zHS1(i) = v(3);
    
    v = projectionPlan([xHS2(i),yHS2(i),zHS2(i)],orig,n);
    xHS2(i) = v(1);yHS2(i) = v(2);zHS2(i) = v(3);
    
    v = projectionPlan([xHS3(i),yHS3(i),zHS3(i)],orig,n);
    xHS3(i) = v(1);yHS3(i) = v(2);zHS3(i) = v(3);
end



xp = projectionPlan([1 0 0],orig,n);
n = n/norm(n);
//xp = xp/norm(xp);
xp = (xp-orig);
xp = xp/norm(xp);
yp = cross(n,xp);

R = [xp;yp;n];

for i = 1:length(xHS)
    v = R*([xHS(i);yHS(i);zHS(i)]-orig');
    xHSp(i) = v(1);yHSp(i) = v(2);zHSp(i) = v(3);
    
    v = R*([xHS1(i);yHS1(i);zHS1(i)]-orig');
    xHSp1(i) = v(1);yHSp1(i) = v(2);zHSp1(i) = v(3);
    
    v = R*([xHS2(i);yHS2(i);zHS2(i)]-orig');
    xHSp2(i) = v(1);yHSp2(i) = v(2);zHSp2(i) = v(3);
    
    v = R*([xHS3(i);yHS3(i);zHS3(i)]-orig');
    xHSp3(i) = v(1);yHSp3(i) = v(2);zHSp3(i) = v(3);
end

WSmiProj(:,:,1) = [xHSp';yHSp']';
WSmiProj(:,:,2) = [xHSp1';yHSp1']';
WSmiProj(:,:,3) = [xHSp2';yHSp2']';
WSmiProj(:,:,4) = [xHSp3';yHSp3']';

Cxy = computeCxy(WSmiProj,[1 0;0 1]);

oo = R'*[Cxy.origin, 0]'+orig';
[xRect,yRect,zRect] = rect3D(oo',n,xp,yp,Cxy.length,Cxy.width);

//plot3d(x,y,z);
plot3d(xRect,yRect,zRect);
scatter3(xHS,yHS,zHS,"markerEdgeColor", "black","markerFaceColor", [0 .8 .8]);
scatter3(xHS1,yHS1,zHS1,"markerEdgeColor", "black","markerFaceColor", [.8 0 .8]);
scatter3(xHS2,yHS2,zHS2,"markerEdgeColor", "black","markerFaceColor", [.8 0 0]);
scatter3(xHS3,yHS3,zHS3,"markerEdgeColor", "black","markerFaceColor", [0 .8 0]);
