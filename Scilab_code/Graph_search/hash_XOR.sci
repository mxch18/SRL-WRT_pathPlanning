function hash_bin = hash_XOR(pt,n,l,origin)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //
    
    //INPUT
    //pt : a 3d vector
    //n: the size of the hash_list
    //l: cell size
    //origin: a reference point to have only positive values for pt
    
    //OUTPUT
    //
    
//----------------------------------------------------------------------------//
    
    s1 = 73856093;s2 = 19349663;s3 = 83492791;
    pt(1) = floor((pt(1)-origin(1))/l);pt(2) = floor((pt(2)-origin(2))/l);pt(3) = floor((pt(3)-origin(3))/l);
    hash_bin = modulo(bitxor(bitxor(pt(2)*s2,pt(3)*s3),pt(1)*s1),n);
    
endfunction
