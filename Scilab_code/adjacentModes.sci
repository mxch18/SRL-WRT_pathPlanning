function [mAdj,ci,ri,diri] = adjacentModes(curMod,lifted_leg,hs_rad,p)
    // Find modes adjacent to initial stance
    // For the moment, only geometric considerations are taken into account 
    
    //INPUT
        // curMod : the current stance/mode
        // lifted_leg : the free leg's tip position before lifting
        // hs_rad : the half-sphere radius
        // p : the footholds
    //OUTPUT
        // mAdj : the adjacent modes according to certain criteria
        
    //Compute inscribed circle center and radius
    curMod_lift = removeRowFromMat(curMod,lifted_leg);
    ci = inscribedCircleCenter(curMod_lift(1,:),curMod_lift(2,:),curMod_lift(3,:));
    ri = inscribedCircleRadius(curMod_lift(1,:),curMod_lift(2,:),curMod_lift(3,:));
    
    //Compute direction of half-sphere
    diri = lifted_leg-ci;
    diri = diri/norm(diri);
    ci = ci+ri*diri;
    
    //Check for adjacency
    for i = 1:size(curMod,1)
        p = removeRowFromMat(p,curMod(i,:)); // this is to avoid finding the initial stance again
    end
    
    mAdj = [];
    k = 0;
    while k<size(p,1) do
        if insideHalfSphere(p($-k,:),ci,hs_rad,diri) then
            mAdj = [mAdj;p($-k,:)];
        end
        k=k+1;
    end
endfunction
