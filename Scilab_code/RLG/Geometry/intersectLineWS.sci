function [bool,multiple,tInterval,distMax] = intersectLineWS(WSmi_R0,shellDesc,lineDesc,tInc)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //
    
    //INPUT
    //WSmi_R0: the workspace approximation
    //shellDesc: a struct containing the shell parameters
    //              *shellDesc.origin
    //              *shellDesc.extRad
    //              *shellDesc.intRad
    //              *shellDesc.axis
    //              *shellDesc.halfAngle
    //lineDesc: struct. The description of the line:
    //              *lineDesc.origin: line "origin"
    //              *lineDesc.direction: line direction
    //tInc: increment on the line exploration
    
    //OUTPUT
    //
    
    //TODO: 
    
//----------------------------------------------------------------------------//
    bool = %F;
//    zMax=%inf;
    distMax=%inf;
    multiple = %F;
    tInterval = [];
    
    //Project all points from WSmi_R0 onto the line. Collect distances
    dim = size(WSmi_R0,1);
    dist = [];
    [tmp,distMax] = projectionDroite(WSmi_R0(1,:),lineDesc.origin,lineDesc.direction);
    
    
    for i = 2:dim
        [tmp,dist(i)] = projectionDroite(WSmi_R0(i,:),lineDesc.origin,lineDesc.direction);
        if dist(i)>distMax then distMax = dist(i); end
    end
    
    t=-abs(distMax); //Because what if the plan normal and the footholds normal are in opposite direction?
                     //plane_ACP gives normal in an undetermined way wrt to direction!!!
                     
    pt = zeros(1,3);
    pt(1) = lineDesc.origin(1)+t*lineDesc.direction(1);
    pt(2) = lineDesc.origin(2)+t*lineDesc.direction(2);
    pt(3) = lineDesc.origin(3)+t*lineDesc.direction(3);
    
    k=1;
    
    if isInShell(shellDesc,pt) then
        bool = %T;
        boolLast = %T;
        boolNow = %T;
//        zMax = abs(pt(3));
        tInterval(k) = t;
        k = k+1;
    else
        boolLast = %F;
        boolNow = %F;
    end
    
    
    while t<abs(distMax)
        t = t+tInc;
        pt(1) = lineDesc.origin(1)+t*lineDesc.direction(1);
        pt(2) = lineDesc.origin(2)+t*lineDesc.direction(2);
        pt(3) = lineDesc.origin(3)+t*lineDesc.direction(3);
        
        if isInShell(shellDesc,pt) then
            bool = %T;
            boolLast = boolNow;
            boolNow = %T;
            if (~boolLast&boolNow)|(boolLast&~boolNow) then
                tInterval(k) = t;
                k = k+1;
            end
        else
            boolLast = boolNow;
            boolNow = %F;
            if (~boolLast&boolNow)|(boolLast&~boolNow) then
                tInterval(k) = t-tInc;
                k = k+1;
            end
        end
    end
    
    if length(tInterval)>2 then
        multiple = %T;
    end
    
endfunction
