function bool = isInShell(shellDesc,point)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //
    
    //INPUT
    //shellDesc: a struct containing the shell parameters
    //              *shellDesc.origin
    //              *shellDesc.extRad
    //              *shellDesc.intRad
    //              *shellDesc.axis
    //              *shellDesc.halfAngle
    //point: the point we want to assess. Row vector.
    
    //OUTPUT
    //bool: true if point belongs in the shell. False otherwise.
    
//----------------------------------------------------------------------------//
    bool = %F;
    
    if isequal(shellDesc.origin,point) then
        if ~shellDesc.intRad then
            bool = %T;
        end
    else
        nNSp = normNoSqrt(point,shellDesc.origin);
        c1 = shellDesc.intRad**2 <= nNSp;
        c2 = nNSp <= shellDesc.extRad**2;
        
        v = point-shellDesc.origin;
        c3 = v*shellDesc.axis'/norm(v)>=cos(shellDesc.halfAngle);
        
        bool=c1&c2&c3;
    end
    
endfunction
