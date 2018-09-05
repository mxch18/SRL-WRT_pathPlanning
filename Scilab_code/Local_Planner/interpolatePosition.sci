function pathP = interpolatePosition(config1P,config2P,s)
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
    pathP = zeros(1,3);
    //Compute cubic polynom coefficients
    p = zeros(4,3);
    
    p(1,:) = config1P;
    p(3,:) = -3*(config1P-config2P);
    p(4,:) = 2*(config1P-config2P);
    
    //Interpolate and store the values
    t = 0;
    k = 1;
    while t<=1
        pathP(k,:) = p(1,:)+p(2,:)*t+p(3,:)*(t**2)+p(4,:)*(t**3);
        k = k +1;
        t = t + 1/s;
    end
    
endfunction
