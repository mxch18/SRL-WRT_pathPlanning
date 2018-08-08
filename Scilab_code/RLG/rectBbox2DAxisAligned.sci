function [rectOrigin,rectL,rectW] = rectBbox2DAxisAligned(points)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //Computes the minimal rectangular planar bounding box of the set of points.
    //The bounding box is axis-aligned with the projection of x0 on the plane(or
    //y0 if x0 is perpendicular)
    
    //INPUT
    //points: the set of points. We assume that the point have already been 
    //projected and transformed into the plane's frame. Therefore points are 2D.
    
    //OUTPUT
    //rectOrigin: the bottom left corner of the bounding box
    //rectL: the bounding box dimension along first axis
    //rectW: the bounding box dimension along second axis
    
//----------------------------------------------------------------------------//
    
    mx = max(points(:,1));
    lx = min(points(:,1));
    
    my = max(points(:,2));
    ly = min(points(:,2));
    
    rectOrigin = [lx,ly];
    rectL = mx-lx;
    rectW = my-ly;
    
endfunction
