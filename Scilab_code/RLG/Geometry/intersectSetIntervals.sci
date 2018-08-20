function [bool,inter] = intersectSetIntervals(intrvSet)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //
    
    //INPUT
    //intrvSet : the cell containing the intervals for each leg
    
    //OUTPUT
    //inter: matrix of intervals
    
//----------------------------------------------------------------------------//
    inter = [];
    
    [bool,interTmp] = intersectMultIntervals(intrvSet(1).entries,intrvSet(2).entries);
    
    if ~bool then
        return;
    end
    
    n = size(intrvSet,2); //number of legs
    
    for i =3:n
        [bool,interTmp] = intersectMultIntervals(intrvSet(i).entries,interTmp);
        if ~bool then
            return;
        end
    end
    
    inter = interTmp;
    
endfunction
