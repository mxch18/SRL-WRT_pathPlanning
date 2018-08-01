function node_num = kd_tree(points,depth,parent_number)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    global cellIn;
    global lcell;
    
    //Description:
    //Computes a kd_tree from the dataset
    
    //INPUT:
    //points : the dataset
    //depth : the current node depth. Root is at depth 0
    //cellIn : a cell array. Global variable. Initially empty (cellIn = cell())
    //lcell : the cell array current size
    //parent_number : the node's parent position/number in the cell array
    
    //OUTPUT:
    //node_num : the node's position/number in the cell array
    
    //INFO:
    //node = struct('parent',0,'leftChild',0,'rightChild',0,'axis',0,'isLeaf',%T,'point',null)
        //parent is the number/position of the parent in the cell array. 0 for root.
        //number is the number/position of the node in the cell array.
        //leftChild is the number/position of the left child in the cell array. -1 if no child.
        //rightChild is the number/position of the right child in the cell array. -1 if no child.
        //axis is the normal to the splitting hyperplane at the node's depth
        //isLeaf indicates if the node is a leaf or not
        //point is the actual k-d point representation
        
//----------------------------------------------------------------------------//
    
    
    //initialize root
    root = struct('parent',0,'number',0,'leftChild',-1,'rightChild',-1,'axis',0,'isLeaf',%T,'point',null);
    
    root.parent = parent_number;
    lcell = lcell+1;
    
    lp = size(points,1);
    if lp<2 then //leaf node
        root.point = points;
        node_num = lcell;
        root.number = node_num;
        cellIn(node_num).entries = root;
        return;
    end
    root.isLeaf = %F;
    
//    nd = ceil(sqrt(lp));
    nd =1;
    dim = size(points,2);
    
    //axis for this split
    axis = 1+modulo(depth,dim-1);
    root.axis = axis;
    //find median along this direction
    medi = randMedian(points,ceil(lp/nd),axis);
    root.point = medi;
    
    node_num = lcell;
    root.number = node_num;
    cellIn(node_num).entries = root;
    
    //remove median from dataset
    points = removeRowFromMat(points,medi);
    //split new dataset in two parts
    points1 = points(find(points(:,axis)>=medi(axis)),:);
    points2 = points(find(points(:,axis)<medi(axis)),:);
    
    //recurse if points
    if size(points1,1) then
//        disp("right"+string(depth));
        cellIn(node_num).entries.rightChild = kd_tree(points1,depth+1,node_num);
    end
    if size(points2,1) then
//        disp("left"+string(depth));
       cellIn(node_num).entries.leftChild = kd_tree(points2,depth+1,node_num);
    end
    
endfunction
