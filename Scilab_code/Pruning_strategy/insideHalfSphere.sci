function ins = insideHalfSphere(point,centre,radius,direction)
    //checks if the point is inside the halfsphere defined by the 3 last args
    vec = point-centre;
    ins = (norm(vec)<=radius)&(vec*direction'>0);
    
endfunction
