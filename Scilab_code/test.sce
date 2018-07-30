//clear;clc;
//getd(".");
//
//close();
////figure;
//r=0.45;
//orig=[1.5 0 0];
//alp=linspace(-%pi/2,%pi/2,40);
//tet=linspace(0,%pi,40);
//direction=[1 1 1];
//direction = direction/norm(direction);
//[xx,yy,zz]=halfSph(orig,r,alp,tet,direction);
//
////[xx1,yy1,zz1]=halfSph(orig,r,linspace(-%pi/2,%pi/2,40),linspace(0,2*%pi,40),[1,1,1]);
//
//plotPts([xx;yy;zz]',2,4,13,13);
//
////plot3d(xx',yy',zz',theta=90,alpha=90);
////hp = get("hdl");
////hp.surface_mode = "off";
////hp.mark_mode = "on";
////hp.mark_style = 2;
////hp.mark_size_unit = "point";
////hp.mark_size = 4;
////hp.ambient_color = [1,0,0];
////hp.mark_foreground = 13;
////hp.mark_background = 13;
////cc=(xx+zz+2)*32;cc1=(xx1-orig(1)+zz1/r+2)*32;
////clf();plot3d1([xx xx1],[yy yy1],list([zz,zz1],[cc cc1]),theta=70,alpha=80,flag=[5,6,3])
//

clear;clearglobal;getd(".");
global cellIn;
cellIn = cell();
global lcell;
lcell = 0;
p = rand(10,3)*10;
kd_tree(p,0,0);

