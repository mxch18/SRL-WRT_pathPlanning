function [bool_success,STANCE_GRAPH,expansion_nodes,single_cndts_node_nb] = plan(INIT_STANCE,DATASET,PARAMS)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //
    
    //INPUT
    //PARAMS: struct containing:
    //          *
    
    //OUTPUT
    //
    
//----------------------------------------------------------------------------//
    bool_success = %F;path = [];
    
//------------------------------Compute normals-------------------------------//
    mprintf("Creating hash-table for %d footholds\n",size(DATASET,1));
    //Construct hash table for dataset
    dataset_hash = list();
    dataset_hash(410) = [];
    dataset_hash(410) = null();
    for i_d_h=1:length(dataset_hash)
        dataset_hash(i_d_h) = [];
    end
    
    for i_d_h=1:size(DATASET,1)
        d_hash_bin = hash_XOR(DATASET(i_d_h,:),length(dataset_hash),PARAMS.cell_size,PARAMS.origin);
        dataset_hash(i_d_h) = [dataset_hash(i_d_h); DATASET(i_d_h,:)];
    end
    mprintf("Hash table created\n")
//--------------------------End of compute normals----------------------------//
    
//--------------------Initialize STANCE_GRAPH---------------------------------//
    new_stances_ini = find_adjacent(INIT_STANCE,DATASET,PARAMS);
    
    metanet_graph = make_graph('stance_graph',1,1,[1],[1]);
    metanet_graph = delete_edges([1 1],metanet_graph);
    
    stance_list = list(struct('stance',INIT_STANCE,'gcost',0,'hcost',0));
    
    stance_list(1).hcost = geometric_cost(INIT_STANCE,PARAMS.goal_stance);
    
    stance_hash = list();
    stance_hash(PARAMS.hash_size+1) = 0;
    stance_hash(PARAMS.hash_size+1) = null();
    
    for i=1:size(stance_ini,2)
        stance_pos(i,:) = stance_ini.pos(i);
    end
    centroid = mean(stance_pos,'r'); //centroid of the initial stance
    hash_bin = hash_XOR(centroid,length(stance_hash),PARAMS.cell_size,PARAMS.origin);
    stance_hash(hash_bin) = [centroid, -1];
    
    edge_list = list();
    
    cost_list = list(struct('node_number',1,'node_cost',stance_list(1).hcost,'expanded',%T));
    
    STANCE_GRAPH = struct('metanet_graph',metanet_graph,'stance_list',stance_list,'edge_list',edge_list,'stance_hash',stance_hash,'cost_list',cost_list)
    
    for i_init = 1:size(new_stances_ini,1)
        [b_add,bool_success,STANCE_GRAPH] = add_stance_to_graph(new_stances_ini(i_init,:),1,STANCE_GRAPH,PARAMS);
    end
//-------------------End of Initialize STANCE_GRAPH---------------------------//
    
//------------------------------Main loop-------------------------------------//
    
    while ~bool_success
//-------------------Pop the single-mode planning candidates------------------//
        i_pop_nb = 1;
        nb_candidates = min(PARAMS.nb,length(STANCE_GRAPH.cost_list));
        single_cndts_node_nb = list();
        for i_pop_nb = 1:nb_candidates
            [STANCE_GRAPH.cost_list,single_cndts_node_nb(i_pop_nb)] = pop_head(STANCE_GRAPH.cost_list);
        end
//---------------End of Pop the single-mode planning candidates---------------//
        
//-------------------------Single-mode planning-------------------------------//
        //single-mode planning for 4-stance
        for i_sg_md=1:nb_candidates
            curr_node_nb = single_cndts_node_nb(i_sg_md).node_number;
            
            curr_cdt = STANCE_GRAPH.stance_list(curr_node_nb); //extracting from stance_list (peep)
            for i_nl4 = 1:PARAMS.nl4
                curr_cdt = single_mode_planning(curr_cdt,PARAMS,4,dataset_hash);
            end
            
            //single-mode planning for 3-stance edge
            curr_children = successors(curr_node_nb,STANCE_GRAPH.metanet_graph);
            if ~isempty(curr_children) then
                ta = curr_node_nb*ones(1,length(curr_children));
                curr_edges = index_from_tail_head(STANCE_GRAPH.metanet_graph,ta,curr_children);
                
                for i_sg_md_edg=1:length(curr_edges)
                    curr_edge_nb = curr_edges(i_sg_md_edg);
                    curr_edge = STANCE_GRAPH.edge_list(curr_edge_nb);
                    
                    if curr_edge.link ~= -1 then
                        curr_edge.stance = STANCE_GRAPH.edge_list(curr_edge.link).stance;
                        curr_edge.roadmap = STANCE_GRAPH.edge_list(curr_edge.link).roadmap;
                    end
                    
                    curr_edge_cost = curr_edge.cost;
                    
                    for i_nl3 = 1:PARAMS.nl3
                        curr_edge = single_mode_planning(curr_edge,PARAMS,3,dataset_hash);
                    end
                end
                
                STANCE_GRAPH.edge_list(curr_edge_nb).cost = curr_edge.cost; //injecting back into edge_list
                STANCE_GRAPH.edge_list(curr_edge.link).roadmap = curr_edge.roadmap;
                
                curr_child_nb = STANCE_GRAPH.metanet_graph.edges(curr_edge_nb).head; //node number of edge end
                STANCE_GRAPH.stance_list(curr_child_nb).hcost = STANCE_GRAPH.stance_list(curr_child_nb).hcost+(curr_edge.cost-curr_edge_cost)/2;
                
            end
            
            STANCE_GRAPH.stance_list(curr_node_nb) = curr_cdt; //injecting back into stance_list
            
            curr_cost = struct('node_number',curr_node_nb,'node_cost',curr_cdt.node_cost,'expanded',curr_cdt.expanded)
            STANCE_GRAPH.cost_list = insert_in_queue(STANCE_GRAPH.cost_list,curr_cost);
        end
//---------------------End of Single-mode planning----------------------------//
        
//----------------------Peep the nodes for expansion--------------------------//
        i_pop_ne = 1;
        k_ne = 1;
        l_cost = length(STANCE_GRAPH.cost_list)
        expansion_nodes = list();
        while (i_pop_ne <= l_cost)&(k_ne <= PARAMS.ne)
            temp_ne = peep_member(STANCE_GRAPH.cost_list,i_pop_ne);
            if ~temp_ne.expanded then
                expansion_nodes(k_ne) = temp_ne;
                k_ne = k_ne + 1;
            end
            i_pop_ne = i_pop_ne + 1;
        end
//--------------------End of Peep the nodes for expansion---------------------//

        bool_success = %T;
    end
endfunction
