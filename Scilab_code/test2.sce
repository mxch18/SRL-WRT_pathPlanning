clear;close();clc;getd(".");

orig = floor(rand(1,3)*10);
//orig = [1,1,1];
n = floor(rand(1,3)*10);
//n = [1,1,1];
[x,y,z] = rect3D(orig,n);

//xtoproj = floor(rand(1,3)*10);
xtoproj = [1 0 0];
xproj = projection(xtoproj,orig,n);

//disp(cross(n,xproj-xtoproj))

//scatter3(xtoproj(1)*ones(1,2),xtoproj(2)*ones(1,2),xtoproj(3)*ones(1,2),50,'red');
//scatter3(xproj(1)*ones(1,2),xproj(2)*ones(1,2),xproj(3)*ones(1,2),50,'green');

alp=linspace(-%pi/2,%pi/2,10);
tet=linspace(0,%pi,10);
rHS = 5;
dirHS = [0 0 1]
ctLift = floor(rand(1,3)*10);
[xHS,yHS,zHS]=halfSph(ctLift,rHS,alp,tet,dirHS);

for i = 1:length(xHS)
    v = projection([xHS(i),yHS(i),zHS(i)],orig,n);
    xHS(i) = v(1);yHS(i) = v(2);zHS(i) = v(3);
end



xp = projection([1 0 0],orig,n);
n = n/norm(n);
//xp = xp/norm(xp);
xp = (xp-orig);
xp = xp/norm(xp);
yp = cross(n,xp);

R = [xp;yp;n];

for i = 1:length(xHS)
    v = R*([xHS(i);yHS(i);zHS(i)]-orig');
    xHSp(i) = v(1);yHSp(i) = v(2);zHSp(i) = v(3);
end

[rectO,rectL,rectW] = rectBbox2DAxisAligned([xHSp,yHSp]);
oo = R'*[rectO, 0]'+orig';
[xRect,yRect,zRect] = rect3D(oo',n,xp,yp,rectL,rectW);

plot3d(x,y,z);
plot3d(xRect,yRect,zRect);
scatter3(xHS,yHS,zHS);
