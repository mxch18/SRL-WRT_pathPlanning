clear;clc;getd("./Local_Planner");


q1 = createQuaternion(0,[1 0 0]);
q2 = createQuaternion(%pi/2,[0 1 0]);
q3 = createQuaternion(-%pi/4,[1 0 1]);
q4 = createQuaternion(-3*%pi/2,[1 1 0]);
q5 = createQuaternion(-%pi/3,[0 0 1]);

vector = [1  1 1];
angle = %pi;

R = matrix_fromAngleVector(angle,vector);

[a,b] = angle_vector_FromMat(R);

disp(a)
disp(b)
