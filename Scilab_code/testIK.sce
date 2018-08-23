IK_target_RLeg_real = [0.069828271865844727, 0.23262149095535278, 0.35102164745330811]; //the foothold for the ith leg, in the leg attachment frame
//disp(IK_target_RLeg);

//FR Leg

l1 = 0.1;
l2 = 0.15;
l3 = 0.3;

xOff = [1 0 0]*0.15/2;
yOff = [0 1 0]*0.3/2;

offset_i = - xOff - yOff;

orient = [-0.7022104 -0.4188790 1.258015];
psi = o(1);
theta = o(2);
phi = o(3);
posi = [-0.0200745 0.0028024 0.1932086];

Rz0 = [cos(psi), -sin(psi), 0;sin(psi), cos(psi) 0;0 0 1];
Rx1 = [1, 0, 0;0, cos(theta), -sin(theta);0 sin(theta) cos(theta)];
Rz2 = [cos(phi), -sin(phi), 0;sin(phi), cos(phi) 0;0 0 1];

R_0_EF = Rz0*Rx1*Rz2;

disp(R_0_EF);

R_Leg_EF = [0 -1 0;-1 0 0;0 0 -1];

IK_target_RLeg = -R_Leg_EF*offset_i' + R_Leg_EF*R_0_EF'*([+2.3750e-1,-4.3146e-1,+1.9097e-2]'-p')

THETA(1) = atan(IK_target_RLeg(2),IK_target_RLeg(1)); //OK

rem = sqrt(IK_target_RLeg(1)**2+IK_target_RLeg(2)**2)-l1;
nc3 = IK_target_RLeg(3)**2+rem**2-l2**2-l3**2;
dc3 = 2*l2*l3;
c3 = nc3/dc3;

if abs(c3)>1 then
    mprintf("IK - NO SOLUTION FOR LEG INVERSE KINEMATICS\n");
    abort;
end
s3 = sqrt(1-c3**2); //ELBOw UP
THETA(3) = -atan(s3,c3);

THETA(2) = -atan(IK_target_RLeg(3),rem)+atan(l3*s3,l2+l3*c3)

P(1) = l1*cos(THETA(1))+l2*cos(THETA(1))*cos(THETA(2))+l3*cos(THETA(1))*cos(THETA(2)+THETA(3));
P(2) = l1*sin(THETA(1))+l2*sin(THETA(1))*cos(THETA(2))+l3*sin(THETA(1))*cos(THETA(2)+THETA(3));
P(3) = -l2*sin(THETA(2))-l3*sin(THETA(2)+THETA(3));

//disp(IK_target_RLeg);
//disp(P);
//disp(THETA*180/%pi);
