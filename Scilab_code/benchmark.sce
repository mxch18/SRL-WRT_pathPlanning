clear;clc;close();getd(".");getd("./kNN");getd("./RLG");getd("./RLG/Geometry");getd("./Pruning_strategy");getd("./Local_Planner");

f1=struct('leg','HL','pos',[-2.3750e-1,-4.3146e-1,+1.9095e-2]);
f2=struct('leg','FL','pos',[-2.3750e-1,+4.3146e-1,+1.9095e-2]);
f3=struct('leg','HR','pos',[+2.3750e-1,-4.3146e-1,+1.9095e-2]);
f4=struct('leg','FR','pos',[+2.3750e-1,+4.3146e-1,+1.9095e-2]);

//f1=struct('leg','HL','pos',[-2.3750e-1,0,-4.3146e-1]);
//f2=struct('leg','FL','pos',[-2.3750e-1,0,+4.3146e-1]);
//f3=struct('leg','HR','pos',[+2.3750e-1,0,-4.3146e-1]);
//f4=struct('leg','FR','pos',[+2.3750e-1,0,+4.3146e-1]);

stance = [f1,f2,f3,f4];

foot_n = [0 0 1];
foot_n = [foot_n;foot_n;foot_n;foot_n];

extRad = 0.55*ones(1,4);
distApiOb = sqrt(0.075**2+0.15**2)*ones(1,4);
intRad = 0*ones(1,4);

//params = struct('extRad',extRad,'distApiOb',distApiOb,'intRad',intRad,'halfAngle',%pi/2,'shellPtsNb',20,'shrink',0.4,'kpxy',5,'tInc',0.04,'baseDimensions',[0.15,0.3],'kpz',5,'aInc',2*%pi/30,'kRot',10,'legLength',[0.1,0.15,0.3],'verbose',%F);
params = struct('extRad',extRad,'distApiOb',distApiOb,'intRad',intRad,'halfAngle',%pi/2,'shellPtsNb',20,'shrink',0.4,'kpxy',5,'tInc',0.04,'baseDimensions',[0.15,0.3],'kpz',5,'aInc',2*%pi/30,'kRz',5,'kRx',5,'legLength',[0.1,0.15,0.3],'verbose',%F);

times = [];
successRate = [];
distance_to_normal = [];

//NORMAL STANCE
p_normal = [0 0 1.49e-1];
q_normal = createQuaternion(0,[0 0 1]);

n= 100;

for i = 1:n
    tic();
//    [p,q,thet,rmat,succ]=RLG(stance,foot_n,params);
    [p,q,thet,rmat,succ]=RLG_Euler(stance,foot_n,params);
    if succ then
        distance_to_normal = [distance_to_normal; distanceBtwConfig([p_normal q_normal],[p q])];
    end
    successRate = [successRate;succ];
    t = toc();
    times = [times; t];
    mprintf('Iteration %d of %d\n',i,n)
end

mprintf('Rate of success over %d tries: %.1f%%',n,100*sum(bool2s(successRate))/n);
