function [bool_add,bool_fin,stance_graph_out] = add_stance_to_graph(STNC,parent_node_nb,stance_graph,params)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //
    
    //INPUT
    //STNC : an array of foothold struct
    //stance_graph: a struct containing:
    //                  *metanet_graph: a graph defining the relation between the stances
    //                  *stance_list: a list of the 4-stance currently in the graph + their cost
    //                  *edge_list: a list of the 3-stance currently in the graph + their cost
    //                  *stance_hash: the hash list containing the binned centroids to try to
    //                          make sure you don't add the same stance 2 times
    //                  *cost_list : a priority queue, updated every time we add new stances
    
    //OUTPUT
    //
    
    //TODO
    //
//----------------------------------------------------------------------------//
    bool_fin = %F
    if isequal(find_centroid(params.goal_stance),find_centroid(STNC)) then
        bool_fin = %T;
        mprintf("Goal stance reached! Hallelujah!\n");
    end
    
    stance_graph_out = struct('metanet_graph',stance_graph.metanet_graph,'stance_list',stance_graph.stance_list,'edge_list',stance_graph.edge_list,..
                              'cost_list',stance_graph.cost_list,'stance_hash',stance_graph.stance_hash);
    
    bool_add = %F;
    
    // Check if stance already exists
    cntr = find_centroid(STNC); //centroid of the current stance. Row vector
    hash_bin = hash_XOR(cntr,length(stance_graph_out.stance_hash),params.cell_size,params.origin);
    
    bool_hash_ok = %T; //does the bucket already exists - because of list implementation
    
    try
        stance_graph_out.stance_hash(hash_bin)
    catch
        bool_hash_ok = %F
    end
    
    if bool_hash_ok then
        for i=1:size(stance_graph_out.stance_hash(hash_bin),1)
            if isequal(cntr,stance_graph_out.stance_hash(hash_bin)(i,1:3)) then
                mprintf("Centroid already exists, not adding stance\n");
                return; //we don't add stances that already exist...
            end
        end
        stance_graph_out.stance_hash(hash_bin)=[stance_graph_out.stance_hash(hash_bin);cntr, -1];
    else
        stance_graph_out.stance_hash(hash_bin) = [cntr, -1]; //add centroid to hash_table
    end
    
    mprintf("Adding stance\n");
    
    node_nb = node_number(stance_graph_out.metanet_graph);
    stance_graph_out.metanet_graph = add_node(stance_graph_out.metanet_graph,[0;0]); //add to graph
    stance_graph_out.metanet_graph = add_edge(parent_node_nb,node_nb+1,stance_graph_out.metanet_graph);
    
    stance_gcost = stance_graph_out.stance_list(parent_node_nb).gcost+1;//number of steps
    stance_hcost = geometric_cost(STNC,params.goal_stance);//cost associated with other factors
    stance_graph_out.stance_list($+1) = struct('stance',STNC,'gcost',stance_gcost,'hcost',stance_hcost); //add to end of stance_list
    
    mprintf("Stance added\n");
    bool_add = %T;
    
    
    mprintf("Adding edge\n");
    edge_nb = edge_number(stance_graph_out.metanet_graph);
    
    new_edge_stance = identify_transition(STNC,stance_graph_out.stance_list(parent_node_nb).stance);
    new_edge_stance_hcost = geometric_cost(new_edge_stance,params.goal_stance);
    // Check if transition already exists
    cntr_edge = find_centroid(new_edge_stance); //centroid of the transition. Row vector
    hash_bin_edge = hash_XOR(cntr_edge,length(stance_graph_out.stance_hash),params.cell_size,params.origin);
    
    bool_hash_edge_ok = %T;
    
    try
        stance_graph_out.stance_hash(hash_bin_edge)
    catch
        bool_hash_edge_ok = %F
    end
    
    if bool_hash_edge_ok then
        for i=1:size(stance_graph_out.stance_hash(hash_bin_edge),1)
            if isequal(cntr_edge,stance_graph_out.stance_hash(hash_bin_edge)(i,1:3)) then
                mprintf("Edge stance already exists, not adding stance to edge_list but linking it somehow lol\n");
                
                exist_edge_nb = stance_graph_out.stance_hash(hash_bin_edge)(i,4);
                
                stance_graph_out.edge_list($+1) = struct('link',exist_edge_nb,'stance',0,'cost',0);
//                stance_graph_out.stance_list(parent_node_nb).hcost = stance_graph_out.edge_list(exist_edge_nb).cost/2 + stance_graph_out.stance_list(parent_node_nb).hcost;
                stance_graph_out.stance_list(node_nb+1).hcost = stance_graph_out.edge_list(exist_edge_nb).cost/2 + stance_graph_out.stance_list(node_nb+1).hcost;
                
                stance_hcost = stance_graph_out.stance_list(node_nb+1).hcost;
                new_cost_elmt = struct('node_number',node_nb+1,'node_cost',stance_gcost+stance_hcost,'expanded',%F);
                stance_graph_out.cost_list = insert_in_queue(stance_graph_out.cost_list,new_cost_elmt);
                
                mprintf("Edge added\n");
                return;
            end
        end
        stance_graph_out.stance_hash(hash_bin_edge) = [stance_graph_out.stance_hash(hash_bin_edge);cntr_edge, edge_nb];
    else
        stance_graph_out.stance_hash(hash_bin_edge) = [cntr_edge, edge_nb]; //add centroid to hash_table
    end
    
    stance_graph_out.edge_list($+1) = struct('link',-1,'stance',new_edge_stance,'cost',new_edge_stance_hcost);
//    stance_graph_out.stance_list(parent_node_nb).hcost = new_edge_stance_hcost/2 + stance_graph_out.stance_list(parent_node_nb).hcost;
    stance_graph_out.stance_list(node_nb+1).hcost = new_edge_stance_hcost/2 + stance_graph_out.stance_list(node_nb+1).hcost;
    
    stance_hcost = stance_graph_out.stance_list(node_nb+1).hcost;
    new_cost_elmt = struct('node_number',node_nb+1,'node_cost',stance_gcost+stance_hcost,'expanded',%F);
    stance_graph_out.cost_list = insert_in_queue(stance_graph_out.cost_list,new_cost_elmt);
    
    mprintf("Edge added\n");
    
endfunction
