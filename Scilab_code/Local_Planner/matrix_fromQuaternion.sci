function mat = matrix_fromQuaternion(q)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //Outputs the interpoled path between two positions with s steps. 
    //Interpolation is cubic
    
    //INPUT
    //config1P : a row vector containing a 3D position
    //config2P : idem
    
    //OUTPUT
    //pathP : a (s x 3) matrix containing the successive values of the parameters
    
    //TODO :
    
//----------------------------------------------------------------------------//
    
    qw = q(1);qx = q(2);qy = q(3);qz = q(4);
    
    mat = [1-2*qy**2-2*qz**2, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw;2*qx*qy+2*qz*qw, 1-2*qx**2-2*qz**2, 2*qy*qz-2*qx*qw;2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx**2-2*qy**2];
    
endfunction
