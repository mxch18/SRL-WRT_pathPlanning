function [stackOut,neighborsOut,distanceOut] = tree_expl(kdtree,root,point,neighborsIn,stackIn,distanceIn)
    //go down the tree to a leaf node
    
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
