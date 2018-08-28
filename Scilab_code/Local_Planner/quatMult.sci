function qOut = quatMult(q1,q2)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //Outputs the quaternion qOut = q1*q2;
    
    //INPUT
    //q1 : quaternion. Row vector
    //q2 : quaternion. Row vector
    
    //OUTPUT
    //qOut: quaternion. Row vector
    
//----------------------------------------------------------------------------//
    
    s = q1(1);
    t = q2(1);
    v = q1(2:4);
    w = q2(2:4);
    
    qOut = zeros(1,4);
    
    qOut(1) = s*t-v*w';
    qOut(2:4) = s*w+t*v+cross(v,w);
    
endfunction
