function xproj = projection(x,orig,n)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //Projects x onto the plane whose normal is n
    
    //INPUT
    //x : the point to be projected. Row vector.
    //orig : the plane's origin
    //n : the plane's normal. Row vector.
    //d : the minimal distance between the plane and the origin
    
    //OUTPUT
    //xproj: the projected point. Row vector.
    
//----------------------------------------------------------------------------//
    
    n = n/norm(n);
    d = norm(n*orig');
    xproj = x - (x*n')*n;
    xproj = xproj + d*n;
    
endfunction
