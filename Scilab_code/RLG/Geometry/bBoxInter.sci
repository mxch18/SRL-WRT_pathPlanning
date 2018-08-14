function [bool,pointsInter] = bBoxInter(bbox,points)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //
    
    //INPUT
    //bbox: the definition of the first bounding box. Struct.
               //bbox.origin: the bottom left corner
               //bbox.v1: one of the bbox direction
               //bbox.v2: the other bbox direction
               //bbox.length: the bbox size in v1 direction
               //bbox.width: the bbox size in v2 direction
    //points: the points for whom we want to check intersection
    
    //OUTPUT
    //bool: true if at least one point of points is in bbox
    //pointsInter: the points of points that belong in bbox
    
//----------------------------------------------------------------------------//
    
    pointsInter = %nan*ones(1,2);
    bool = %F;
    
    l = size(points,1);
    k=1;
    
    for i=1:l
        
        pp = points(i,:)-bbox.origin;
        s1 = pp*(bbox.v1)';
        s2 = pp*(bbox.v2)';
        
        c1 = (0<s1)&(s1<bbox.length);
        c2 = (0<s2)&(s2<bbox.width);
        
        if c1&c2 then
            pointsInter(k,:) = points(i,:);
            k = k+1;
        end
    end
    
    if ~isnan(pointsInter(1)) then
        bool=%T;
    end
    
endfunction
