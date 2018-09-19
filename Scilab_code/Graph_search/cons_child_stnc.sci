function stance_list = cons_child_stnc(STNC,pts_classified)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //
    
    //INPUT
    //STNC: array of foothold structs
    //pts_classified: cell containing the points. The cell keeps the order of
    //                the stance
    
    
    //OUTPUT
    //stance_list: a (nx4) matrix of stances. n is the added length of the
    //elements of pts_classified
    
//----------------------------------------------------------------------------//
    l = size(STNC,2);
    stance_list = STNC;
    
    for i = 1:size(pts_classified,1)
//        disp(i)
        for j = 1:size(pts_classified(i).entries,1)
//            disp(j)
            new_fthld = struct('leg',STNC(i).leg,'pos',pts_classified(i).entries(j,:));
            if i==1 then
                new_stnc = [new_fthld,STNC((i+1):l)']
            elseif i==size(pts_classified,1) then
                new_stnc = [STNC(1:(i-1))',new_fthld];
            else
                new_stnc = [STNC(1:(i-1))',new_fthld,STNC((i+1):l)'];
            end
//            disp(new_stnc)
            stance_list = [stance_list;new_stnc];
        end
    end
    
    l2 = size(stance_list,1);
    stance_list = stance_list(2:l2,:);
    
endfunction
