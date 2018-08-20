function [bool,intersection] = intersectMultIntervals(intv1,intv2)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //
    
    //INPUT
    //intv1: matrix of intervals
    //intv2: matrix of intervals
    
    //OUTPUT
    //intersect: matrix of intervals
    
//----------------------------------------------------------------------------//
    l1 = size(intv1,1);
    l2 = size(intv2,1);
    
    intersection = [];
    bool = %F;
    k = 1;
    
    for i=1:l1
        for j=1:l2
            [dump,temp] = intersectTwoIntervals(intv1(i,:),intv2(j,:));
            if dump then
                bool = %T;
                intersection(k,:) = temp';
                k = k+1;
            end
        end
        if dump then
            k = k+1;
        end
    end
    
endfunction
