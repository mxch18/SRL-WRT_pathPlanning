clear;clc;getd("./Local_Planner");

f1=struct('leg','HL','pos',[-2.3750e-1,-4.3146e-1,+1.9095e-2]);
f2=struct('leg','FL','pos',[-2.3750e-1,+4.3146e-1,+1.9095e-2]);
f3=struct('leg','HR','pos',[+2.3750e-1,-4.3146e-1,+1.9095e-2]);
f4=struct('leg','FR','pos',[+2.3750e-1,+4.3146e-1,+1.9095e-2]);

stance = [f1,f2,f3,f4];

p1 = [0 0 0];
q1 = createQuaternion(0,[1 0 0]);

c1 = [p1 q1];

p2 = [0 0 0.37];
q2 = createQuaternion(3*%pi/20,[0 0 1]);

c2 = [p2 q2];

extRad = 0.55*ones(1,4);
distApiOb = sqrt(0.075**2+0.15**2)*ones(1,4);
intRad = 0*ones(1,4);

params = struct('extRad',extRad,'distApiOb',distApiOb,'intRad',intRad,'halfAngle',%pi/2,'shellPtsNb',20,'shrink',0.2,'kpxy',5,'tInc',0.04,'baseDimensions',[0.15,0.3],'kpz',5,'aInc',2*%pi/30,'kRz',5,'kRx',5,'legLength',[0.1,0.15,0.3],'verbose',%T);

leg_theta = localPlanner(c1,c2,50,params,stance);

leg_file = mopen("./leg_theta.txt","w");

for i = 1:size(leg_theta,1)
    mfprintf(leg_file,"%f %f %f %f %f %f %f %f %f %f %f %f\n",leg_theta(i,1),leg_theta(i,2),leg_theta(i,3),leg_theta(i,4),leg_theta(i,5),leg_theta(i,6),leg_theta(i,7),..
    leg_theta(i,8),leg_theta(i,9),leg_theta(i,10),leg_theta(i,11),leg_theta(i,12));
end

mclose(leg_file);
