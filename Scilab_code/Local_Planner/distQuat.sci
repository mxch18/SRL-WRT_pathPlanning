function d = distQuat(q1,q2)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //Outputs the quaternion qOut = q1*q2;
    
    //INPUT
    //q1 : quaternion. Row vector
    //q2 : quaternion. Row vector
    
    //OUTPUT
    //d : distance between the two quaternions according to this metric:
    //          d = 1 - abs(q1.q2)
    //see paper [] for reference
//----------------------------------------------------------------------------//
    
    d = 1 - abs(q1*q2');
    
endfunction
