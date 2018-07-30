function [bool,disOut,index] = isBetter(disIn,queryPoint,newPoint)
    
    
    bool = %F;
    ld = size(disOut,1);
    disQN = normNoSqrt(queryPoint,newPoint);
    disOut = disIn;
    dmax = max(disIn);
    if disQN<dmax then
        bool = %T;
        index = find(disIn==dmax,1);
        disOut(index) = disQN;
    end
    
endfunction
