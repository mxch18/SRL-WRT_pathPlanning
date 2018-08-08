function [xproj,dist] = projectionDroite(x,orig,n)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //Projects x onto the line whose direction is n
    
    //INPUT
    //x : the point to be projected. Row vector.
    //orig : the line's "origin"
    //n : the line's direction. Row vector.
    
    //OUTPUT
    //xproj: the projected point. Row vector.
    //dist: the distance to the origin.
    
//----------------------------------------------------------------------------//
    
    n = n/norm(n);
    xproj = ((xproj-orig)*n')*n + orig;
    
endfunction
