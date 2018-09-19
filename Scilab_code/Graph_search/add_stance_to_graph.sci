function [bool,metanet_graph_out,stance_list_out,stance_hash_out] = add_stance_to_graph(STNC,parent_node_nb,metanet_graph,stance_list,stance_hash,params)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //
    
    //INPUT
    //STNC : an array of foothold struct
    //metanet_graph: a graph defining the relation between the stances
    //stance_list: a list of the stance currently in the graph + cost
    //stance_hash: the hash list containing the binned centroids to try to
    //             make sure you don't add the same stance 2 times
    //cost_list : a priority queue, updated every time we add new stances
    
    //OUTPUT
    //
    
    //TODO
    //Clarify code, cuz rn it's garbage
//----------------------------------------------------------------------------//
    metanet_graph_out = metanet_graph;
    stance_list_out = stance_list;
    stance_hash_out = stance_hash;
    bool = %F;
    
    // Check if stance already exists
    for i=1:size(STNC,2)
        stance_pos(i,:) = STNC.pos(i);
    end
    
    centroid = mean(stance_pos,'r'); //centroid of the current stance. Row vector
//    centroid_norm = norm(centroid);
//    hash_bin = quantize(centroid_norm,qtz_step);
    hash_bin = hash_XOR(centroid,length(stance_hash_out),params.cell_size,params.origin)
    
    if hash_bin>length(stance_hash_out) then //stance not in graph for sure
        mprintf("Adding stance\n");
        
        stance_hash_out(hash_bin) = centroid; //add centroid to hash_table
        
        node_nb = node_number(metanet_graph_out);
        metanet_graph_out = add_node(metanet_graph_out,[0;0]); //add to graph
        metanet_graph_out = add_edge(parent_node_nb,node_nb+1,metanet_graph_out);
        
//        cost_STNC = stance_cost(STNC,parent_node_nb,
        stance_gcost = stance_list_out(parent_node_nb).gcost+1;
        stance_hcost = geometric_cost(STNC,params.goal_stance);
        stance_list_out($+1) = struct('stance',STNC,'gcost',stance_gcost,'hcost',stance_hcost); //add to end of stance_list
        
        mprintf("Stance added\n");
        bool = %T;
    else
        bool_hash_ok = %T;
        
        try
            stance_hash_out(hash_bin);
        catch //if there's an error, that means stance_hash(hash_bin) is undefined
            bool_hash_ok = %F;
            
            mprintf("Adding stance\n");
            
            stance_hash_out(hash_bin) = centroid;
            
            node_nb = node_number(metanet_graph_out);
            metanet_graph_out = add_node(metanet_graph_out,[0;0]); //add to graph
            metanet_graph_out = add_edge(parent_node_nb,node_nb+1,metanet_graph_out);
            
//          cost_STNC = stance_cost(STNC,parent_node_nb,
            stance_gcost = stance_list_out(parent_node_nb).gcost+1;
            stance_hcost = geometric_cost(STNC,params.goal_stance);
            stance_list_out($+1) = struct('stance',STNC,'gcost',stance_gcost,'hcost',stance_hcost); //add to end of stance_list
            
            mprintf("Stance added\n");
            bool = %T;
        end
        
        if bool_hash_ok then
            for i=1:size(stance_hash_out(hash_bin),1)
                if isequal(centroid,stance_hash_out(hash_bin)(i,:)) then
                    mprintf("Centroid already exists, not adding stance\n");
                    return;
                end
            end
            mprintf("Adding stance\n");
            stance_hash_out(hash_bin)=[stance_hash_out(hash_bin);centroid];
            
            node_nb = node_number(metanet_graph_out);
            metanet_graph_out = add_node(metanet_graph_out,[0;0]); //add to graph
            metanet_graph_out = add_edge(parent_node_nb,node_nb+1,metanet_graph_out);
            
//          cost_STNC = stance_cost(STNC,parent_node_nb,
            stance_gcost = stance_list_out(parent_node_nb).gcost+1;
            stance_hcost = geometric_cost(STNC,params.goal_stance);
            stance_list_out($+1) = struct('stance',STNC,'gcost',stance_gcost,'hcost',stance_hcost); //add to end of stance_list
            
            mprintf("Stance added\n");
            bool = %T;
        end
        
    end
    
    
endfunction
