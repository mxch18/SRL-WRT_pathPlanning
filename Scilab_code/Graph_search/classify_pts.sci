function pts_classified = classify_pts(pts_in_ball,STNC)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //Classify points according to their distance to the points in STNC
    
    //INPUT
    
    //OUTPUT
    
//----------------------------------------------------------------------------/
    
    pts_classified = cell(size(STNC,2),1);
    
    objectives = STNC.pos; //list
    closest = -1;
    dist_to_clos = %inf;
    distance_to_obj = 0;
    
    for i = 1:size(pts_in_ball,1)
        closest = -1;
        dist_to_clos = %inf;
        for j = 1:length(objectives)
            if isequal(pts_in_ball(i,:),objectives(j)) then
                continue; //skip the points equal to the current footholds
            else
                distance_to_obj = normNoSqrt(objectives(j),pts_in_ball(i,:));
                if distance_to_obj<dist_to_clos then
                    dist_to_clos = distance_to_obj;
                    closest = j;
                end
            end
        end
//        mprintf("Closest is %d\n",closest)
        pts_classified.entries(closest) = [pts_classified.entries(closest); pts_in_ball(i,:)];
    end
    
endfunction
