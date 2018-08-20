function intervalMatrix = createAngleInterval(interval_Array)
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
    l = length(interval_Array);
    
    if l==1 then
        intervalMatrix = [-%pi, %pi];
    else
        m = (1/4)*(2*l+(-1)^(l)+1);
        intervalMatrix = %pi*ones(m,2);
        for i=1:l
            intervalMatrix((1/4)*(2*i+(-1)^(i+1)+1),2-modulo(i,2)) = interval_Array(i);
        end
    end
    
endfunction
