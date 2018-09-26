function elements = LSH_query(hash_LSH,metric,query,k,radius)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //Approximate k-NN query based on Locality-Sensitive-Hashing. Outputs node
    //numbers
    
    //INPUT
    //hash_LSH: struct
    //              *hash_LSH.dimension: the dimension of the vectors we're hashing
    //              *hash_LSH.nb: the number of hash functions
    //              *hash_LSH.size: the size of the hash tables
    //              *hash_LSH.hashing_functions: struct
    //                      *array: [a11 a12 ... a1k b1;a21 a22 ... a2k b2;...]
    //                      *prime_factor: a big prime number
    //              *hash_LSH.hash_tables: list of list. each element of the list is a hash-table.
    //metric: the metric we want to use
    //query: a row vector representing
    //k: the number of neighbors we want
    
    //OUTPUT
    //
    
//----------------------------------------------------------------------------//
    elements = [];
    
    if length(query) ~= hash_LSH.dimension then
        mprintf("Query is of the wrong dimension");
        return;
    end
    
    p_f = hash_LSH_out.hashing_functions.prime_factor;
    
    k_inc = 1;
    i_nb = 1
    
    while (k_inc<k)&(i_nb<=hash_LSH.nb)
        
        x_i = hash_LSH.hashing_functions.array(i_nb,1:hash_LSH.dimension);
        b_i = hash_LSH.hashing_functions.array(i_nb,hash_LSH.dimension+1);
        h_i = modulo(floor(p_f*(new_element*x_i'+b_i)),hash_LSH.size);
        
        i_hash = 1;
        
        while (k_inc<k)&(i_hash<=length(hash_LSH.hash_tables(i_nb)(h_i)))
            if metric(query,hash_LSH.hash_tables(i_nb)(h_i)(i_hash,1:($-1)))<= radius then
                pos = members(elements, hash_LSH.hash_tables(i_nb)(h_i)(i_hash,$), "rows");
                if isempty(find(pos==1)) then
                    elements(k_inc) = hash_LSH.hash_tables(i_nb)(h_i)(i_hash,$);
                    k_inc = k_inc+1;
                end
            end
            i_hash = i_hash+1;
        end
        
        i_nb = i_nb+1;
    end
    
endfunction
