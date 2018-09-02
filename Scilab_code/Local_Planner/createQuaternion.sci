function q = createQuaternion(alpha,u)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //Outputs the quaternion qOut = q1*q2;
    
    //INPUT
    //alpha: angle of rotation (radian)
    //u: axis of rotation
    
    //OUTPUT
    //q: the quaternion of unit magnitude representing the rotation of alpha around u
//----------------------------------------------------------------------------//
    u = u/norm(u);
    
    q = [cos(alpha/2), sin(alpha/2)*u(1), sin(alpha/2)*u(2), sin(alpha/2)*u(3)];
    q = q/norm(q);
    
endfunction
