//Test planning
clear;clc;close();getd(".");getd("./kNN");getd("./RLG");getd("./RLG/Geometry");getd("./Pruning_strategy");getd("./Local_Planner");getd("./Graph_search");getd("./Planner");

filen = mopen("./points.txt","r");
p = zeros(1,3)
i = 1

while ~meof(filen) do
    p(i,1:3) = mfscanf(1,filen,"%f %f %f");
    i = i+1;
end

mclose(filen);

i1=struct('leg','HL','pos',[-2.3344e-1, -4.9237e+0, -4.7664e-1]);
i2=struct('leg','FL','pos',[-2.3184e-1, -4.0805e+0, -4.6890e-1]);
i3=struct('leg','HR','pos',[+2.3070e-1, -4.9226e+0, -4.8433e-1]);
i4=struct('leg','FR','pos',[+2.3249e-1, -4.0774e+0, -4.6157e-1]);

stance_ini = [i1,i2,i3,i4];

g1=struct('leg','HL','pos',[-2.2538e-1, -3.411e+0, +2.2829e-1]);
g2=struct('leg','FL','pos',[-2.23e-1, -2.59e+0, +3.74e-1]);
g3=struct('leg','HR','pos',[+2.3223e-1, -3.4162e+0, +2.3764e-1]);
g4=struct('leg','FR','pos',[+2.25e-1, -2.5963e+0, +3.685e-1]);

stance_goal = [g1,g2,g3,g4];

ref_pt = min(p,'r');
cell_size = 0.25;

extRad = 0.55*ones(1,4);
distApiOb = sqrt(0.075**2+0.15**2)*ones(1,4);
intRad = 0*ones(1,4);

function_set = struct('query_ws',in_ball_naive,'classifier',classify_pts);

params = struct('ball_radius',0.6,..
                'origin',ref_pt,..
                'cell_size',cell_size,..
                'goal_stance',stance,..
                'function_set',function_set,..
                'hash_size',1013,..
                'nb',5,..
                'ne',1,..
                'nl4',3,..
                'nl3',1,..
                'extRad',extRad,..
                'distApiOb',distApiOb,..
                'intRad',intRad,..
                'halfAngle',%pi/2,..
                'shellPtsNb',20,..
                'shrink',0.2,..
                'kpxy',5,..
                'tInc',0.04,..
                'baseDimensions',[0.15,0.3],..
                'kpz',5,..
                'aInc',2*%pi/30,..
                'kRz',2,..
                'kRx',2,..
                'legLength',[0.1,0.15,0.3],..
                'verbose',%F,..
                'metric',distanceBtwConfig,..
                'neigh_nb',5,..
                'PRM_radius',5,..
                'hash_LSH_nb',15,..
                'hash_LSH_size',607);



