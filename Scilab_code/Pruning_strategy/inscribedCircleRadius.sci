function r = inscribedCircleRadius(p1,p2,p3)
    //Computes the inscribed circle radius
    n12 = norm(p2-p1);
    n23 = norm(p3-p2);
    n31 = norm(p1-p3);
    
    sumn = (n12+n23+n31)/2;
    
    r=sqrt(sumn-n12)*(sumn-n31)*(sumn-n23)/sumn;
endfunction
