clear;close();clc;

l1 = 0.1;
l2 = 0.15;
l3 = 0.3;

tet1 = linspace(-%pi,%pi,30);
tet2 = linspace(-%pi,%pi,30);
tet3 = linspace(-%pi,%pi,30);

[TET1,TET2,TET3]=ndgrid(tet1,tet2,tet3);

X = l3.*cos(TET1).*cos(TET2+TET3)+l2.*cos(TET1).*cos(TET2)+l1.*cos(TET1);
Y = l3.*sin(TET1).*cos(TET2+TET3)+l2.*sin(TET1).*cos(TET2)+l1.*sin(TET1);
Z = l3.*sin(TET2+TET3)+l2.*sin(TET2);

figure;
f=gcf();

set(gca(),"auto_clear","off")
for i=1:length(tet1)
    plot3d2(X(:,:,i),Y(:,:,i),Z(:,:,i))
end

