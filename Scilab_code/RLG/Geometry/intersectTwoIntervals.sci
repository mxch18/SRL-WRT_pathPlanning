function [bool,intersection] = intersectTwoIntervals(intv1,intv2)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //
    
    //INPUT
    //
    
    //OUTPUT
    //
    
//----------------------------------------------------------------------------//
    intersection = [];
    bool = %F;
    
    if isempty(intv1)|isempty(intv2) then
        return;
    end
    
    if (intv2(1)>intv1(2))|(intv1(1)>intv2(2)) then
        return;
    else
        bool = %T;
        intersection(1) = max(intv1(1),intv2(1));
        intersection(2) = min(intv1(2),intv2(2));
    end
    
endfunction
