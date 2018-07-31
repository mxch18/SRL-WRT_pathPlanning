function neighbors = bruteForceNeighbors(points,point,k)
    
    wp = size(points,2);
    for i=1:k
        lp = size(points,1);
        minDis = normNoSqrt(point,points(1,:));
        neighbors(i,:) = zeros(1,wp);
        
        for j =2:lp
            curDis = normNoSqrt(point,points(j,:));
            if (curDis~=0)&(curDis<minDis) then
                minDis = curDis;
                neighbors(i,:) = points(j,:);
            end
        end
        
        points = removeRowFromMat(points,neighbors(i,:));
        
    end
    
    
endfunction
