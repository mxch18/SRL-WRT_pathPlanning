function neighbors = kNN(kdtree,k,point)
    //Outputs the k-nearest neighbors of an input point. Uses a k-d tree structure as input.
    
    //INPUT:
    //kdtree : the k-d tree structure. It is a cell array.
    //k : the numbers of neighbors we want
    //point : the point from which we want neighbors
    
    //OUTPUT:
    //neighbors: a (k x 3) matrix containing the coordinates of the k nearest neighbors
    
    // distance field
    distance = zeros(k,1);
    //init neighbors with root point
    for i=1:k
        neighbors(i,:) = kdtree(1).entries.point;
//        distance(i) = normNoSqrt(point-kdtree(1).entries.point);
        distance(i) = 1000000000000;
    end
    
    //init current node
    curNode = kdtree(1).entries;
    //init stack
    stack = list();
//    stack = stackInsert(stack,curNode);
    
    //first exploration of the tree
    //[stackOut,neighborsOut,distanceOut] = tree_expl(kdtree,root,point,neighborsIn,stackIn,distanceIn)
    [stack,neighbors,distance] = tree_expl(kdtree,curNode,point,neighbors,stack,distance);
    disp("Done with first exploration");
//    [curNode,stack] = stackPop(stack); //pop the leaf node, throw it away
    
    //reverse exploration of tree
    //[stackOut,distanceOut,neighborsOut] = tree_rev_expl(kdtree,stackIn,point,distanceIn,neighborsIn)
    [stack,distance,neighbors] = tree_rev_expl(kdtree,stack,point,distance,neighbors);
    disp("Done with reverse exploration");
    
endfunction
