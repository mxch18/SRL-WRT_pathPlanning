function Cxy = computeCxy(WSmiProj,axis)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //
    
    //INPUT
    //WSmiProj: the projection of the approximated workspaces onto the foothold 
    //plane. This is a hypermatrix:
    //            - WSmiProj(:,:,i): projection of the i-th workspace on the 
    //                               foothold plane
    //axis: the axes on which Cxy is aligned
    
    //OUTPUT
    //Cxy: the intersection of the projected approximated workspaces. It is a 
    //bbox structure. All its elements are in the plane's frame (2D)
    
    //TODO: Iteration on the computation of intersection so that we can limit
    //      further the error on the intersection. Keep the current pattern,
    //      but make the computation of intersection between two sets iterative
    //      As of now, maybe put a shrinking factor on bbox size?
    //      A large part of the error in intersection comes from the axis-
    //      oriented nature of the bounding boxes.
    
//----------------------------------------------------------------------------//
    
    dim = size(WSmiProj);
    
    Cxy = struct('origin',%nan,'v1',%nan,'v2',%nan,'length',%nan,'width',%nan);
    
    if (dim(3)<3)|(dim(3)>4) then
        disp('computeCxy : Invalid stance.');
        return;
    end
    
    //Compute bounding boxes of first two legs
    for i =1:dim(3)
        [rO,rL,rW] = rectBbox2DAxisAligned(WSmiProj(:,:,i));
        WSmiProjBBox(i) = struct('origin',rO,'v1',axis(1,:),'v2',axis(2,:),'length',rL,'width',rW);
    end
    
    //Compute intersection
    [curBool1,curInt1] = bBoxInter(WSmiProjBBox(1),WSmiProj(:,:,2));
    [curBool2,curInt2] = bBoxInter(WSmiProjBBox(2),WSmiProj(:,:,1));
    if ~curBool1|~curBool2 then
        mprintf("computeCxy: Leg 1 and 2 workspaces do not intersect!\n");
        return;
    end
    
    curInt=[curInt1;curInt2];
    [currO,currL,currW] = rectBbox2DAxisAligned(curInt);
    curBBox = struct('origin',currO,'v1',axis(1,:),'v2',axis(2,:),'length',currL,'width',currW);
    
    for i=3:dim(3)
        [curBool1,curInt1] = bBoxInter(curBBox,WSmiProj(:,:,i));
        [curBool2,curInt2] = bBoxInter(WSmiProjBBox(i),curInt);
        
        if ~curBool1|~curBool2 then
            mprintf("computeCxy: Leg %d and previous leg workspaces intersection do not intersect!\n",i);
            return;
        end
        
        curInt=[curInt1;curInt2];
        [currO,currL,currW] = rectBbox2DAxisAligned(curInt);
        curBBox = struct('origin',currO,'v1',axis(1,:),'v2',axis(2,:),'length',currL,'width',currW);
    end
    
    Cxy = curBBox;
    
    //shrinking factor to account for error in intersection.
    //doesn't work because origin is bottom left corner -> we can put that in sampling
//    Cxy.length = shrink*Cxy.length;
//    Cxy.width = shrink*Cxy.width;
    
    
endfunction
