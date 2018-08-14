function [x,y,z] = rect3D(orig,n,xd,yd,l,w)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //Outputs the coordinates of the four corners of a lxw rectangle with 
    //surface normal n
    
    //INPUT
    //orig : one of the rectangle's corner. Line vector.
    //n : the rectangle surface normal
    //x : the plane x direction. We assume orthonormal base.
    //y : the plane y direction. We assume orthonormal base.
    //l : the rectangle's length
    //w : the rectangle's width
    
    //OUTPUT
    //x,y,z: the coordinates of the rectangle's corners. Column vectors.
    
//----------------------------------------------------------------------------//
    
    rhs = argn(2);
    
    x = zeros(4,1);
    y = zeros(4,1);
    z = zeros(4,1);
    
    points = zeros(4,3);
    
    if rhs==2 then
        //we just want to draw a big plane
        l=50;
        w=50;
        
        if ~n(2)&~n(1) then 
            v1 = [1 0 0];
        else
            v1 = [-n(2),n(1),0];
        end
        
        v2 = cross(n,v1);
        
        v1 = v1/norm(v1);
        v2 = v2/norm(v2);
    elseif rhs == 6 then
        //this is the case when we want to draw a small rectangle like a bbox
        xd = xd/norm(xd);
        yd = yd/norm(yd);
        v1=xd;
        v2=yd;
    else
        disp('Inappropriate number of arguments');
        abort;
    end
    
    points(1,:) = orig;
    points(2,:) = points(1,:)+l*v1;
    points(3,:) = points(2,:)+w*v2;
    points(4,:) = points(1,:)+w*v2;
    x = points(:,1);
    y = points(:,2);
    z = points(:,3);
    
endfunction
