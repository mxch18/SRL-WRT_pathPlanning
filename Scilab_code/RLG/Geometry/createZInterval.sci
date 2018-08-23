function intervalMatrix = createZInterval(interval_Array,d)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //
    
    //INPUT
    //
    
    //OUTPUT
    //
    
//----------------------------------------------------------------------------//
    intervalMatrix = [];
    interval_Array = unique(interval_Array);
    l = length(interval_Array);
    
//    disp(zMax);disp(d);
    
    if l==1 then
        intervalMatrix = [interval_Array, interval_Array+d];
    else
        m = (1/4)*(2*l+(-1)^(l)+1);
        intervalMatrix = d*ones(m,2);
        for i=1:l
            intervalMatrix((1/4)*(2*i+(-1)^(i+1)+1),2-modulo(i,2)) = interval_Array(i);
        end
    end
    
endfunction
