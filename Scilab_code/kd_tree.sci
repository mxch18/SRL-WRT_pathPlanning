function root = kd_tree(points,depth)
    //Computes a kd_tree from the dataset
    //INPUT:
    //points : the dataset
    //depth : the current node depth. Root is at depth 0.
    
    //OUTPUT:
    //root: the tree's root
    //the root variable contains all information on the rest of the tree.
    
    //node = struct('leftChild',null,'rightChild',null,'point',null)
        //leftChild is a tree or nothing if leaf
        //rightChild is a tree or nothing if leaf
        //point is the actual k-d point representation
    
    //axis = 1+modulo(depth,k-1)
    
    //initialize root
    root = struct('leftChild',null,'rightChild',null,'axis',null,'isLeaf',%T,'point',null);
    
    lp = size(points,1);
    if lp<2 then //leaf node
        root.point = points;
        return;
    end
    
    root.isLeaf = %F;
//    disp("hehe balek")
    nd = ceil(sqrt(lp));
//    nd =1;
    dim = size(points,2);
    
    
    //axis for this split
    axis = 1+modulo(depth,dim-1);
    root.axis = axis;
    //find median along this direction
    medi = randMedian(points,ceil(lp/nd),axis);
    root.point = medi;
    
    //remove median from dataset
    points = removeRowFromMat(points,medi);
    //split new dataset in two parts
    points1 = points(find(points(:,axis)>=medi(axis)),:);
    points2 = points(find(points(:,axis)<medi(axis)),:);
    
    //recurse if points
    if size(points1,1) then
//        disp("right"+string(depth));
        root.rightChild = kd_tree(points1,depth+1);
    end
    if size(points2,1) then
//        disp("left"+string(depth));
       root.leftChild = kd_tree(points2,depth+1);
    end
    
endfunction
