clear;clc;getd(".");
close();

l2 = 0.15;
l3 = 0.3;
fact = 1.5;
rHS = fact*(l2+l3);

filen = mopen("./points.txt","r");
disp("File opened")
p = zeros(1,3)
i = 1

while ~meof(filen) do
    p(i,1:3) = mfscanf(1,filen,"%f %f %f");
    i = i+1;
end

mclose(filen);
disp("File closed")

pIni = zeros(4,3); // initial position
pIni = [-0.25,-5,-4.5;-0.25,-4.2,-4.5;0.25,-5,-4.5;0.25,-4.2,-4.5]

// Find modes adjacent to initial stance with left front leg lifted

FL = [-0.25,-4.2,-4.5];

[pAdj,ctLift,rtLift,dirHS] = adjacentModes(pIni,FL,rHS,p);

p_tmp = p;

for i=1:size(pAdj,1)
    p_tmp = removeRowFromMat(p_tmp,pAdj(i,:));
end

// Plot everything
plotPts(p_tmp,2,4,10,10);
plotPts(pAdj,0,6,13,13);
plotPts(pIni,11,8,5,5);

// Draw half-sphere
draw=1; //0 if not drawing
if draw then
    alp=linspace(-%pi/2,%pi/2,20);
    tet=linspace(0,%pi,20);
    [xHS,yHS,zHS]=halfSph(ctLift,rHS,alp,tet,dirHS);
    plotPts([xHS;yHS;zHS]',0,1,13,13);
end
