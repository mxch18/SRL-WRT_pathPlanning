function med = randMedian(points,nbRanPts,axis)
    //Computes the median of uniformly randomly selected points from a dataset
    //INPUT:
    //points : the dataset
    //nbRanPts : the number of points we will select for the computation of the median
    //axis : the axis we will check the median for
    
    //OUTPUT:
    //med : the median point wrt to the axis
    
//    tmp = zeros(1,3);
    // select nbRanPts to compute median
    for i = 1:nbRanPts
        lp = size(points,1);
        k = floor(1+(lp-1)*rand());
        tmp(i,:) = points(k,:);
        points = removeRowFromMat(points,tmp(i,:));
    end
    
    tmp_col = tmp(:,axis);
    tmp_col_sort = gsort(tmp_col);
    
    index1 = ceil(nbRanPts/2);
    val = tmp_col_sort(index1);
    
    indMed = find(tmp_col==val);
    
    med = tmp(indMed,:);
    
endfunction
