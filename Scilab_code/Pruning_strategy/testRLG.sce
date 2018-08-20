clear;clc;close();getd(".");getd("./kNN");getd("./RLG");getd("./RLG/Geometry");getd("./Pruning_strategy");

f1=struct('leg','HL','pos',[-0.25,-5,-4.5]);
f2=struct('leg','FL','pos',[-0.25,-4.2,-4.5]);
f3=struct('leg','HR','pos',[0.25,-5,-4.5]);
f4=struct('leg','FR','pos',[0.25,-4.2,-4.5]);
stance = [f1,f2,f3,f4];

stance_pos_list=stance(:).pos;

for i=1:size(stance,2)
    stance_pos_array(i,:) = stance_pos_list(i);
end

[a,b,c] = plane_ACP(stance_pos_array);
[x,y,z] = rect3D(c,a);

foot_n = [0 0 1];
foot_n = [foot_n;foot_n;foot_n;foot_n];

extRad = 0.55*ones(1,4);
intRad = 0*ones(1,4);

params = struct('extRad',extRad,'intRad',intRad,'halfAngle',%pi/2,'shellPtsNb',20,'shrink',1,'kpxy',5,'tInc',0.04,'baseDimensions',[0.15,0.3],'kpz',5,'aInc',2*%pi/30);

tic();
[d,e,f,h,i,j,h]=RLG(stance,foot_n,params);
disp(toc());

oo = f'*[d.origin, 0]'+c';
//oo(2)=oo(2)+4.5
[xRect,yRect,zRect] = rect3D(oo',a,i,j,d.length,d.width);

//plot3d(xRect,yRect,zRect);
//scatter3(e(:,1,1),e(:,2,1),e(:,3,1),"markerEdgeColor", "black","markerFaceColor", [0 .8 .8]);
//scatter3(e(:,1,2),e(:,2,2),e(:,3,2),"markerEdgeColor", "black","markerFaceColor", [.8 0 .8]);
//scatter3(e(:,1,3),e(:,2,3),e(:,3,3),"markerEdgeColor", "black","markerFaceColor", [.8 0 0]);
//scatter3(e(:,1,4),e(:,2,4),e(:,3,4),"markerEdgeColor", "black","markerFaceColor", [0 .8 0]);
