clear;clc;getd(".");
close();

l2 = 0.15;
l3 = 0.3;
fact = 1;
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
ctLift=FL;
p_tmp = p;

if length(pAdj) then
    for i=1:size(pAdj,1)
        p_tmp = removeRowFromMat(p_tmp,pAdj(i,:));
    end
end


//Real leg workspace

l1 = 0.1;
l2 = 0.15;
l3 = 0.3;

tet1 = linspace(-%pi,%pi,10);
tet2 = linspace(-%pi,%pi,10);
tet3 = linspace(-%pi,%pi,10);

[TET1,TET2,TET3]=ndgrid(tet1,tet2,tet3);

X = l3.*cos(TET1).*cos(TET2+TET3)+l2.*cos(TET1).*cos(TET2)+l1.*cos(TET1)+ctLift(2);
Y = l3.*sin(TET1).*cos(TET2+TET3)+l2.*sin(TET1).*cos(TET2)+l1.*sin(TET1)+ctLift(1);
Z = l3.*sin(TET2+TET3)+l2.*sin(TET2)+ctLift(3);

// Plot everything
plotPts(p_tmp,2,4,10,10);
plotPts(pIni,11,8,5,5);
if length(pAdj) then
    plotPts(pAdj,0,6,13,13);
end


for i=1:length(tet1)
    plot3d3(Y(:,:,i),X(:,:,i),Z(:,:,i))
end

a = gca();
a.zoom_box=[-1,-6,1,-3,-6,-3.5]

// Draw half-sphere
draw=1; //0 if not drawing
if draw then
    alp=linspace(-%pi/2,%pi/2,10);
    tet=linspace(0,%pi,10);
    [xHS,yHS,zHS]=halfSph(ctLift,rHS,alp,tet,dirHS);
    //plotPts([xHS;yHS;zHS]',0,1,13,13);
    plot3d3(xHS',yHS',zHS',alpha=35,theta=45,flag=[30,30])
end
