function [nvect,d, ori] = plane_ACP(X)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //Computes the description of the LS-fit plane to the set of points.
    
    //INPUT
    //X: the set of points. 
    
    //OUTPUT
    //nvect: the plane normal. Row vector.
    //d: the plane distance to origin
    //ori: the plane's frame origin.
    
//----------------------------------------------------------------------------//
    nvect = zeros(1,3);
    d = 0;
    ori = zeros(1,3);
    
    [n,l] = size(X);
    
    if n<3 then
        disp('plane_ACP: At least 3 points needed in dataset. Returning default values.');
        return;
    end
    
    G = [mean(X(:,1)),mean(X(:,2)),mean(X(:,3))];
    
    ori = G;
    
    Xp = X - ones(n,1)*G;
    
    Mcov = zeros(3,3);
    
    for i = 1:n
        Mcov = Mcov + Xp(i,:)'*Xp(i,:);
    end
    
    Mcov = (1/n)*Mcov;
    
    [evals,vp] = spec(Mcov);
    
    nvect = evals(:,1)';
//    res = vp(1)/n;
    d = nvect*G';
    
endfunction
