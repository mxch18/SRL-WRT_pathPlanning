function trans_stnc = identify_transition(STNC1,STNC2)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //
    
    //INPUT
    //
    
    //OUTPUT
    //
    
//----------------------------------------------------------------------------//
    leg = -1;
    
    for i = 1:length(STNC1)
        if ~isequal(STNC1.pos(i),STNC2.pos(i)) then
            leg = i;
            break;
        end
    end
    
    trans_stnc = STNC1(1,[1:(i-1), (i+1):length(STNC1)]);
    
endfunction
