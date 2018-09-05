function pathR = interpolateRotation(config1Q,config2Q,s,thresh)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //Outputs the interpoled path between two orientation with s steps. 
    //Interpolation is SLERP cubic
    
    //INPUT
    //config1Q : a row vector containing a quaternion
    //config2Q : idem
    //assumed unit-magnitude
    //s : the number of steps

    
    //OUTPUT
    //pathR : a (s x 4) matrix containing the successive values of the parameters

    
    //TODO :
    
//----------------------------------------------------------------------------//
    pathR = zeros(1,4);
    
    dot = config1Q*config2Q'; //cos(omega)
    
    if dot<0 then
        config1Q = -config1Q;
        dot = -dot;
    end
    
    if dot > thresh then //linear interpolation, as the two quaternion are so "close"
        t = 0;
        k = 1;
        while t <=1
            pathR(k,:) = config1Q*(1-t)+config2Q*t;
            pathR(k,:) = pathR(k,:)/norm(pathR(k,:));
            k = k+1;
            t = t+1/s;
        end
    else //cubic slerp
        p1 = zeros(1,4);
        p1(1) = 1; p1(2) = 0; p1(3) = -3; p1(4) = 2;
        p1p = poly(p1,'x',"coeff");
        
        p2 = zeros(1,4);
        p2(1) = 0; p2(2) = 0; p2(3) = 3; p2(4) = -2;
        p2p = poly(p2,'x',"coeff");
        
        t = 0;
        k = 1;
        omega = acos(dot);
        somega = sin(omega);
        
        while t <= 1
            pathR(k,:) = (config1Q*sin(horner(p1p,t)*omega) + config2Q*sin(horner(p2p,t)*omega))/somega;
            k = k+1;
            t = t+1/s;
        end
    end
    
endfunction
