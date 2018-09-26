clear;clc;getd("./Local_Planner");

f1=struct('leg','HL','pos',[-2.3750e-1,-4.3146e-1,+1.9095e-2]);
f2=struct('leg','FL','pos',[-2.3750e-1,+4.3146e-1,+1.9095e-2]);
f3=struct('leg','HR','pos',[+2.3750e-1,-4.3146e-1,+1.9095e-2]);
f4=struct('leg','FR','pos',[+2.3750e-1,+4.3146e-1,+1.9095e-2]);
//f3=struct('leg','HR','pos',[]);

stance = [f1,f2,f3,f4];

p1 = [0 0 +1.4900e-1];
q1 = createQuaternion(0,[0 0 1]);
t1 = [30, 60, -120]*%pi/180;

//c1 = [p1 q1];
c1 = [p1 q1 t1];

p2 = [0 0.05 0.25];
ang = 45*%pi/180;
q2 = createQuaternion(ang,[1 1 1]);
t2 = [0, 0, -45]*%pi/180;
//ang = -%pi/3+rand()*2*%pi/3;
//q2 = createQuaternion(ang,rand(1,3));

//c2 = [p2 q2];
c2 = [p1 q1 t2];

extRad = 0.55*ones(1,4);
distApiOb = sqrt(0.075**2+0.15**2)*ones(1,4);
intRad = 0*ones(1,4);

params = struct('extRad',extRad,'distApiOb',distApiOb,'intRad',intRad,'halfAngle',%pi/2,'shellPtsNb',20,'shrink',0.2,'kpxy',5,'tInc',0.04,'baseDimensions',[0.15,0.3],'kpz',5,'aInc',2*%pi/30,'kRz',5,'kRx',5,'legLength',[0.1,0.15,0.3],'verbose',%T);

//disp(distanceBtwConfig(c1,c2,3))

p_ini = [0 0 +1.49e-1];
px_inc = linspace(-0.5,0.5,50);
py_inc = linspace(-0.5,0.5,50);
pz_inc = linspace(0,0.5,50);

i_x = 1;
success_x = [];
dist_x = [];

i_y = 1;
success_y = [];
dist_y = [];

i_z = 1;
success_z = [];
dist_z = [];

c1 = [p_ini q1]
stance_type1 = 4;

while i_x<=length(px_inc)
    p2 = [px_inc(i_x) p_ini(2) p_ini(3)];
    c2 = [p2 q1];
    dist_x = [dist_x; distanceBtwConfig(c1,c2,stance_type1)];
    [dump,success_x_i] = localPlanner(c1,c2,50,params,stance,stance_type1);
    success_x = [success_x, success_x_i];
    i_x = i_x+1;
end

while i_y<=length(py_inc)
    p2 = [p_ini(1) py_inc(i_y) p_ini(3)];
    c2 = [p2 q1];
    dist_y = [dist_y; distanceBtwConfig(c1,c2,stance_type1)];
    [dump,success_y_i] = localPlanner(c1,c2,50,params,stance,stance_type1);
    success_y = [success_y, success_y_i];
    i_y = i_y+1;
end

while i_z<=length(pz_inc)
    p2 = [p_ini(1) p_ini(2) pz_inc(i_z)];
    c2 = [p2 q1];
    dist_z = [dist_z; distanceBtwConfig(c1,c2,stance_type1)];
    [dump,success_z_i] = localPlanner(c1,c2,50,params,stance,stance_type1);
    success_z = [success_z, success_z_i];
    i_z = i_z+1;
end

ind_x = find(success_x==%t)
min_x = px_inc(ind_x(1));
max_x = px_inc(ind_x($));

ind_y = find(success_y==%t)
min_y = py_inc(ind_y(1));
max_y = py_inc(ind_y($));

ind_z = find(success_z==%t)
min_z = pz_inc(ind_z(1));
max_z = pz_inc(ind_z($));

mprintf("\nx range : %.4f to %.4f\ny range : %.4f to %.4f\nz range : %.4f to %.4f\n",min_x,max_x,min_y,max_y,min_z,max_z);

radius = 5;
w_t = radius*(((max_x-min_x)/2)**2 + ((max_y-min_y)/2)**2 + ((max_z-min_z)/2)**2)**-1;

i_swing1 = 1;
dist_swing1 = [];

i_swing2 = 1;
dist_swing2 = [];

i_swing3 = 1;
dist_swing3 = [];

t_ini = [30, 60, -120]*%pi/180;
t1_inc = linspace(-180,180,100);
t2_inc = linspace(-180,180,100);
t3_inc = linspace(-180,180,100); 

c1 = [p1 q1 t_ini];
stance_type2 = 3;

while i_swing1<=length(t1_inc)
    t1 = [t1_inc(i_swing1), 60, -120]*%pi/180;
    c2 = [p1 q1 t1];
    dist_swing1 = [dist_swing1; distanceBtwConfig(c1,c2,stance_type2)];
    i_swing1 = i_swing1+1;
end

while i_swing2<=length(t2_inc)
    t2 = [30, t2_inc(i_swing2), -120]*%pi/180;
    c2 = [p1 q1 t2];
    dist_swing2 = [dist_swing2; distanceBtwConfig(c1,c2,stance_type2)];
    i_swing2 = i_swing2+1;
end

while i_swing3<=length(t3_inc)
    t3 = [30, 60, t3_inc(i_swing3)]*%pi/180;
    c2 = [p1 q1 t3];
    dist_swing3 = [dist_swing3; distanceBtwConfig(c1,c2,stance_type2)];
    i_swing3 = i_swing3+1;
end

w_s = radius*mean([max(dist_swing1),max(dist_swing2),max(dist_swing3)])**-1;

q1=createQuaternion(0,[1 0 0])
q2 = createQuaternion(%pi/2,[1 0 0])
w_r = radius/distQuat(q1,q2);

//[leg_theta,success] = localPlanner(c1,c2,50,params,stance,3);

//leg_file = mopen("./leg_theta.txt","w");
//
//for i = 1:size(leg_theta,1)
//    mfprintf(leg_file,"%f %f %f %f %f %f %f %f %f %f %f %f\n",leg_theta(i,1),leg_theta(i,2),leg_theta(i,3),leg_theta(i,4),leg_theta(i,5),leg_theta(i,6),leg_theta(i,7),..
//    leg_theta(i,8),leg_theta(i,9),leg_theta(i,10),leg_theta(i,11),leg_theta(i,12));
//end
//
//mclose(leg_file);
//
