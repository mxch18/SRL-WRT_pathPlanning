function cntr = find_centroid(STNC)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //
    
    //INPUT
    //
    
    //OUTPUT
    //
    
//----------------------------------------------------------------------------//
    
    for i=1:size(STNC,2)
        stance_pos(i,:) = STNC.pos(i);
    end
    
    cntr = mean(stance_pos,'r');
    
endfunction
