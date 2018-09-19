function ADJ_STNC_LIST = find_adjacent(STNC,DATA_STRUC,PARAMS_PRN,FUNCTION_SET)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //
    
    //INPUT
    //
    
    //OUTPUT
    //ADJ_STNC_LIST: an array containing the adjacent stances to STNC
    
//----------------------------------------------------------------------------//
    
    ADJ_STNC_LIST = [];
    
    //Query points belonging to workspace approximation
    pts_reachable = [];
    pts_reachable = FUNCTION_SET.query_ws(STNC,DATA_STRUC,PARAMS_PRN); //check the reachable points according to the ball pruning rule
    
    //Classify them according to a certain function
    pts_classified = cell(length(STNC),1);
    pts_classified = FUNCTION_SET.classifier(pts_reachable,STNC);
    
    //Construct the child stances
    ADJ_STNC_LIST = cons_child_stnc(STNC,pts_classified);
    
endfunction
