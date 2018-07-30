function n = normNoSqrt(p1,p2)
    //Computes the squared norm of the vector defined by p2-p1.
    //INPUT
    
    //OUTPUT
    
    lp = length(p1);
    n = 0;
    for i = 1:lp
        n = n + (p2(i)-p1(i))**2;
    end
    
endfunction
