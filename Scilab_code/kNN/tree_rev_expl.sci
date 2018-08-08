function [stackOut,distanceOut,neighborsOut] = tree_rev_expl(kdtree,stackIn,point,distanceIn,neighborsIn)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //Unwind the tree exploration to check for possible better points on opposite sides of splitting hyperplanes.
    
    //INPUT:
    //kdtree: the kdtree structure
    //stackIn: a list of the nodes visited so far
    //point: the point whom we look neighbors for
    //distanceIn: the current distance to the nearest neighbors
    //neighborsIn: the current nearest neighbors
    
    //OUTPUT:
    //stackOut: the stack after exploring the tree
    //distanceOut: the updated distances to nearest neighbors
    //neighborsOut: the updated nearest neighbors
    
//----------------------------------------------------------------------------//
    
    stackOut = stackIn;
    neighborsOut = neighborsIn;
    distanceOut = distanceIn;
    
    if ~size(stackIn) then
        return;
    end
    
    disp("Starting with stack of size "+string(size(stackIn)))
    
    [curNode,stackIn] = stackPop(stackIn);
    
    str ="[";
    for u=1:length(point)
        str = str+string(point(u))+",";
    end
    
    str = str+"]";
    
    stackTmp = list();
    
    disp("Popped the node n°"+string(curNode.number)+", representing point :"+str);
    
    while curNode.parent //not at root
        disp("The new node parent is "+string(curNode.parent));
        parNode = kdtree(curNode.parent).entries;
        
//        [bool,distanceIn,index] = isBetter(distanceIn,point,parNode.point);
//        if bool then
//            neighborsIn(index,:)=root.point;
//        end
        
        if abs(point(parNode.axis)-parNode.point(parNode.axis)) < max(distanceIn) then
            //we intersect, so we have to check the other side for closer points
            //first we need to actually know which way is the other side
            //then we can explore the correct subtree
            if (parNode.leftChild ~= -1)&(parNode.leftChild ~= curNode.number) then
                //this means that the unexplored subtree is on the left
                disp("Exploring uncharted left tree!");
//                disp(kdtree(parNode.leftChild).entries)
//                [stackOut,neighborsOut,distanceOut] = tree_expl(kdtree,root,point,neighborsIn,stackIn,distanceIn)
                [stackTmp,neighborsIn,distanceIn] = tree_expl(kdtree,kdtree(parNode.leftChild).entries,point,neighborsIn,stackTmp,distanceIn);
            elseif (parNode.rightChild ~= -1)&(parNode.rightChild ~= curNode.number) then
                //this means that the unexplored subtree is on the right
                disp("Exploring uncharted right tree!");
                [stackTmp,neighborsIn,distanceIn] = tree_expl(kdtree,kdtree(parNode.rightChild).entries,point,neighborsIn,stackTmp,distanceIn);
            end
            
            [curNode,stackIn] = stackPop(stackIn);
            
        else
            [curNode,stackIn] = stackPop(stackIn);
        end
        
    end
    
    
    
    stackOut = stackIn;
    neighborsOut = neighborsIn;
    distanceOut = distanceIn;
    
endfunction
