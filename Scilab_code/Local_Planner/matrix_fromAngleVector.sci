function Rmat = matrix_fromAngleVector(angle,vector)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //Outputs the rotation matrix according to Rodrigues formula
    
    //INPUT
    //
    
    //OUTPUT
    //
    
    //TODO :
    
//----------------------------------------------------------------------------//
    
    vector = vector/norm(vector);
    ux = vector(1);
    uy = vector(2);
    uz = vector(3);
    
    P = [ux**2 ux*uy ux*uz;ux*uy uy**2 uy*uz;ux*uz uy*uz uz**2];
    I = eye(3,3);
    Q = [0 -uz uy;uz 0 -ux;-uy ux 0];
    
    c = cos(angle);
//    disp(c)
    s = sin(angle);
//    disp(s)
    Rmat = P+c*(I-P)+s*Q;
    
endfunction
