function ct = inscribedCircleCenter(p1,p2,p3)
    //Computes the inscribed circle center of the triangle defined by the input
    n12 = norm(p2-p1);
    n23 = norm(p3-p2);
    n31 = norm(p1-p3);
    
    sumn = n12+n23+n31;
    
    ct = n12*p3+n23*p1+n31*p2;
    ct = ct/sumn;
endfunction
