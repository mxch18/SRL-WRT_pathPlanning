function hash_LSH_out = LSH_build(new_element,node_nb,hash_LSH)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //
    
    //INPUT
    //hash_LSH: struct
    //              *hash_LSH.dimension: the dimension of the vectors we're hashing
    //              *hash_LSH.nb: the number of hash functions
    //              *hash_LSH.size: the size of the hash tables
    //              *hash_LSH.hashing_functions: struct
    //                      *array: [a11 a12 ... a1k;a21 a22 ... a2k;...]
    //                      *prime_factor: a big prime number
    //              *hash_LSH.hash_tables: list of list. each element of the list is a hash-table.
    //new_element: a row vector
    
    //OUTPUT
    //
    
//----------------------------------------------------------------------------//
    
    hash_LSH_out = hash_LSH;
    
    if length(new_element) ~= hash_LSH_out.dimension then
        mprintf("Element is of the wrong dimension");
        return;
    end
    
    p_f = hash_LSH_out.hashing_functions.prime_factor;
    
    for i = 1:hash_LSH_out.nb
        x_i = hash_LSH_out.hashing_functions.array(i,1:hash_LSH_out.dimension);
//        b_i = hash_LSH_out.hashing_functions.array(i,hash_LSH_out.dimension+1);
        h_i = modulo(floor(p_f*(new_element*x_i')),hash_LSH_out.size);
        hash_LSH.hash_tables(i)(h_i) = [hash_LSH.hash_tables(i)(h_i); new_element node_nb];
    end
    
endfunction
