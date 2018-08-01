function [bool,disOut,index] = isBetter(disIn,queryPoint,newPoint)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //Compares query point distance to a new point, and then updates distance field accordingly.
    //It is used in the context of k-NN search.
    
    //INPUT:
    //disIn: the input distance field. Represents the current best distances to neighbors
    //queryPoint: the point whom we're looing for neighbors
    //newPoint: the point we assess for better proximity
    
    //OUTPUT:
    //bool: a boolean stating if newPoint is a closer neighbor.
    //disOut: an updated distance field
    //index: the ranking of the newPoint wrt to his distance to queryPoint
    
    //TODO : Further test the behavior of this function
    
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
