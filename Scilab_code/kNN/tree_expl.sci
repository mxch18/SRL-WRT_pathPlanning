function [stackOut,neighborsOut,distanceOut] = tree_expl(kdtree,root,point,neighborsIn,stackIn,distanceIn)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //Explore the tree with binary search
    
    //INPUT:
    //kdtree: the kdtree structure
    //root: the point at which we start the exploration
    //point: the point whom we look neighbors for
    //neighborsIn: the current nearest neighbors
    //stackIn: a list of the nodes visited so far
    //distanceIn: the current distance to the nearest neighbors
    
    //OUPUT:
    //stackOut: the stack after exploring the tree
    //neighborsOut: the updated nearest neighbors
    //distanceOut: the updated distances to nearest neighbors
    
//----------------------------------------------------------------------------//
    
//    disp(root)
    stackIn = stackInsert(stackIn,root);
    
    [bool,distanceIn,index] = isBetter(distanceIn,point,root.point);
    if bool then
        neighborsIn(index,:)=root.point;
    end
    
    while ~root.isLeaf
        
        if (root.leftChild==-1)|(root.rightChild==-1) then
            if (root.leftChild==-1) then
                root = kdtree(root.rightChild).entries;
            elseif (root.rightChild==-1) then
                root = kdtree(root.leftChild).entries;
            end
        elseif point(root.axis) < root.point(root.axis) then
            root = kdtree(root.leftChild).entries;
        else
            root = kdtree(root.rightChild).entries;
        end
        
        stackIn = stackInsert(stackIn,root);
        
        [bool,distanceIn,index] = isBetter(distanceIn,point,root.point);
        if bool then
            neighborsIn(index,:)=root.point;
        end
        
    end
    
    stackOut = stackIn;
    neighborsOut = neighborsIn;
    distanceOut = distanceIn;
endfunction
