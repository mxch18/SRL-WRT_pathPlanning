function [bool,disOut,index] = isBetter(disIn,queryPoint,newPoint)
    
    
    bool = %F;
    index = -1;
    disOut = disIn;
    
    disQN = normNoSqrt(queryPoint,newPoint);
    
    if ~disQN then
        return;
    end

    dmax = max(disIn);
    if disQN<dmax then
        bool = %T;
        index = find(disIn==dmax,1);
        disOut(index) = disQN;
    end
    
endfunction
