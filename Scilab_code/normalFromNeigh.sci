function nvect = normalFromNeigh(X)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //From a set of 3-d points X, outputs the normal to the least-square plan fitting the set.
    
    //INPUT:
    //X: the set of 3-d points
    
    //OUTPUT:
    //nvect: the normal
    
//----------------------------------------------------------------------------//
    
    [n,l] = size(X);
    
    G = [mean(X(:,1)),mean(X(:,2)),mean(X(:,3))];
    
    Xp = X - ones(n,1)*G;
    
    Mcov = zeros(3,3);
    
    for i = 1:n
        Mcov = Mcov + Xp(i,:)'*Xp(i,:);
    end
    
    Mcov = (1/n)*Mcov;
    
    [evals,vp] = spec(Mcov);
    
    nvect = evals(:,1);
    
endfunction
