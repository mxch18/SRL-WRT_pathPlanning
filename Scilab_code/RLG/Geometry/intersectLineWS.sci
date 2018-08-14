function [bool,zInterval] = intersectLineWS(WSmi_R0,shellDesc,lineDesc,tInc)
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
    
//----------------------------------------------------------------------------//
    bool = %F;
    zInterval = [];
    
    //Project all points from WSmi_R0 onto the line. Collect distances
    dim = size(WSmi_R0,1);
    dist = [];
    [tmp,distMax] = projectionDroite(WSmi_R0(1,:),lineDesc.origin,lineDesc.direction);
    
    
    for i = 2:dim
        [tmp,dist(i)] = projectionDroite(WSmi_R0(i,:),lineDesc.origin,lineDesc.direction);
        if dist(i)>distMax then distMax = dist(i); end
    end
    
    t=-abs(distMax); //Because what if the plan normal and the footholds normal are in opposite direction?
                //plane_ACP only gives normal in an undetermined way wrt to direction!!!
    pt = zeros(1,3);
    zValid = []
    k=1;
    oldState = %F;
    newState = %F;
    
    //oldState and newState will only give desirable results if the workspace is a CONVEX SET.
    //if the set is not convex, there is no other choice than to explore all of the line
    
    while t<abs(distMax) & (newState|(~oldState&~newState))
        pt(1) = lineDesc.origin(1)+t*lineDesc.direction(1);
        pt(2) = lineDesc.origin(2)+t*lineDesc.direction(2);
        pt(3) = lineDesc.origin(3)+t*lineDesc.direction(3);
        if isInShell(shellDesc,pt) then
            oldState = newState;
            newState = %T;
            zValid(k) = pt(3);
            k = k+1;
        else
            oldState = newState;
            newState = %F;
        end
        t = t+tInc;
    end
    
    if ~isempty(zValid) then
        bool = %T;
        zInterval=[min(zValid(1),zValid($)),max(zValid(1),zValid($))];
    end
    
endfunction
