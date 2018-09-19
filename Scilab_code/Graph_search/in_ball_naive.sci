function pts_in_ball = in_ball_naive(STNC,DATA_STRUC,PARAMS_PRN)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //Outputs the points from dataset that belong to the ball defined in PARAMS
    
    //INPUT
    //PARAMS: a struct containing the radius of the ball
    //          *PARAMS.ball_radius
    //dataset: the set of points
    
    //OUTPUT
    //
    
//----------------------------------------------------------------------------//
    
    pts_in_ball= [];
    
    for i=1:size(STNC,2)
        stance_pos(i,:) = STNC.pos(i);
    end
    
    centroid = mean(stance_pos,'r'); //centroid of the current stance
    
    for i = 1:size(DATA_STRUC,1)
        if normNoSqrt(centroid,DATA_STRUC(i,:)) <= PARAMS_PRN.ball_radius**2 then
            pts_in_ball = [pts_in_ball;DATA_STRUC(i,:)];
        end
    end
    
endfunction
