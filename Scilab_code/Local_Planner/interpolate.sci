function path = interpolate(config1,config2,s)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //Outputs a geometric path between config1 and config2 with s steps
    
    //INPUT
    //config1 : row vector. The configuration of a rigid body in 3D
    //              *config(1:3) = position
    //              *config(4:7) = quaternion
    //config2 : idem. 
    
    //OUTPUT
    //path : a (s x 7) matrix containing the successive values of the parameters
    
    //TODO :
    
//----------------------------------------------------------------------------//
    
    path = [interpolatePosition(config1(1:3),config2(1:3),s) interpolateRotation(config1(4:7),config2(4:7),s,0.99995)];
    
endfunction
