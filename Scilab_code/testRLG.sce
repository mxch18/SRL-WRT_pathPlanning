clear;clc;close();getd(".");getd("./kNN");getd("./RLG");getd("./RLG/Geometry");getd("./Pruning_strategy");getd("./Local_Planner");

f1=struct('leg','HL','pos',[-2.3750e-1,-4.3146e-1,+1.9095e-2]);
f2=struct('leg','FL','pos',[-2.3750e-1,+4.3146e-1,+1.9095e-2]);
f3=struct('leg','HR','pos',[+2.3750e-1,-4.3146e-1,+1.9095e-2]);
f4=struct('leg','FR','pos',[+2.3750e-1,+4.3146e-1,+1.9095e-2]);

//f1=struct('leg','HL','pos',[-2.3750e-1,0,-4.3146e-1]);
//f2=struct('leg','FL','pos',[-2.3750e-1,0,+4.3146e-1]);
//f3=struct('leg','HR','pos',[+2.3750e-1,0,-4.3146e-1]);
//f4=struct('leg','FR','pos',[+2.3750e-1,0,+4.3146e-1]);
//
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
distApiOb = sqrt(0.075**2+0.15**2)*ones(1,4);
intRad = 0*ones(1,4);

params = struct('extRad',extRad,'distApiOb',distApiOb,'intRad',intRad,'halfAngle',%pi/2,'shellPtsNb',20,'shrink',0.2,'kpxy',5,'tInc',0.04,'baseDimensions',[0.15,0.3],'kpz',5,'aInc',2*%pi/30,'kRz',5,'kRx',5,'legLength',[0.1,0.15,0.3],'verbose',%T);

//tic();
[p,o,thet,rmat,succ]=RLG(stance,foot_n,params);
//disp(footPlane_Rmat*footPlane_Rmat')
//disp(toc());
//disp(rmat)

toDeg = 180/%pi;

if succ then
    filen = mopen("./robot_state.txt","w");
    mfprintf(filen,"%f %f %f\n",p(1),p(2),p(3));
//    mfprintf(filen,"%f %f %f\n",o(1),o(2),o(3));
    for i = 1:3
        mfprintf(filen,"%f %f %f\n",rmat(i,1),rmat(i,2),rmat(i,3));
    end
    for i = 1:size(thet,1)
        mfprintf(filen,"%f %f %f\n",thet(i,1),thet(i,2),thet(i,3));
    end
    
    for i=1:size(stance,2)
        mfprintf(filen,"%f %f %f\n",stance(i).pos(1),stance(i).pos(2),stance(i).pos(3));
    end
    
    mclose(filen);
end

//oo = f'*[d.origin, 0]'+c';
//oo(2)=oo(2)+4.5
//[xRect,yRect,zRect] = rect3D(oo',a,i,j,d.length,d.width);

//plot3d(xRect,yRect,zRect);
//scatter3(e(:,1,1),e(:,2,1),e(:,3,1),"markerEdgeColor", "black","markerFaceColor", [0 .8 .8]);
//scatter3(e(:,1,2),e(:,2,2),e(:,3,2),"markerEdgeColor", "black","markerFaceColor", [.8 0 .8]);
//scatter3(e(:,1,3),e(:,2,3),e(:,3,3),"markerEdgeColor", "black","markerFaceColor", [.8 0 0]);
//scatter3(e(:,1,4),e(:,2,4),e(:,3,4),"markerEdgeColor", "black","markerFaceColor", [0 .8 0]);
