function invQ = invQuat(q)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //Outputs the inverse of the quaternion q
    
    //INPUT
    //config1 : row vector. The configuration of a rigid body in 3D
    //              *config(1:3) = position
    //              *config(4:7) = quaternion
    //config2 : idem. 
    
    //OUTPUT
    //path : a (s x 7) matrix containing the successive values of the parameters
    
    //TODO :
    
//----------------------------------------------------------------------------//
    invQ = zeros(1,4);
    
    den = q(1)**2+q(2)**2+q(3)**2+q(4)**2;
    invQ(1) = q(1)/den;
    invQ(2:4) = -q(2:4)/den;
    
endfunction
