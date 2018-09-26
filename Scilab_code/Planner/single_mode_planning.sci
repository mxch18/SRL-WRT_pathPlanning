function STANCE_LIST_ELMT_out = single_mode_planning(STANCE_LIST_ELMT,PARAMS,stance_type,dataset_hash)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //Samples a new configuration, adds it to the roadmap, connects it to the
    //eligible existing milestones
    
    //INPUT
    //STANCE_LIST_ELMT: an element of the stance list. struct containing:
    //              *STANCE_LIST_ELMT.stance
    //              *STANCE_LIST_ELMT.gcost
    //              *STANCE_LIST_ELMT.hcost
    //              *STANCE_LIST_ELMT.roadmap: a struct containing:
    //                  *roadmap.meta_graph
    //                  *roadmap.config_list
    //                  *roadmap.edge_list
    //                  *roadmap.hash_LSH
    //PARAMS:
    //     *PARAMS.PRM_radius
    //dataset_hash: the hash table of the dataset, useful to compute the normals
    
    //OUTPUT
    //
    
//----------------------------------------------------------------------------//
    STANCE_LIST_ELMT_out = STANCE_LIST_ELMT;
    
    NORMALS = [];
    
    for i_normal = 1:size(STANCE_LIST_ELMT.stance,2)
        candidates = [];
        n_hash_bin = hash_XOR(STANCE_LIST_ELMT.stance(i_normal).pos,length(dataset_hash),PARAMS.cell_size,PARAMS.origin);
        candidates = dataset_hash(n_hash_bin);
        while size(candidates,1) < 3 
            n_hash_bin_shake = hash_XOR(STANCE_LIST_ELMT.stance(i_normal).pos+rand(1,3),length(dataset_hash),PARAMS.cell_size,PARAMS.origin);
            if n_hash_bin_shake ~= n_hash_bin then
                n_hash_bin = n_hash_bin_shake;
                candidates = [candidates;dataset_hash(n_hash_bin_shake)];
            end
        end
        normal_i = plane_ACP(candidates);
        normal_i = normal_i/norm(normal_i);
        NORMALS(i_normal,:) = normal_i;
    end
    
    [P,Q,THETA,SUCCESS,NB_TRY] = RLG_Euler(STANCE_LIST_ELMT.stance,NORMALS,PARAMS);
    
    STANCE_LIST_ELMT_out.hcost = STANCE_LIST_ELMT_out.hcost+NB_TRY;
    
    if ~SUCCESS then
        mprintf("Failed to sample a new configuration\n");
        return;
    end
    
    new_config = [P, Q];
//    stance_type = 4;
    if stance_type == 3 then
        swing_leg = -%pi*ones(1,3)+2*%pi*rand(1,3);
        new_config = [P, Q, swing_leg];
//        THETA = [THETA; swing_leg];
    end
    
    mprintf("Adding configuration\n");
    
    node_nb = node_number(STANCE_LIST_ELMT_out.roadmap.meta_graph);
    STANCE_LIST_ELMT_out.roadmap.meta_graph = add_node(STANCE_LIST_ELMT_out.roadmap.meta_graph,[0;0]); //add to graph
    
    STANCE_LIST_ELMT_out.roadmap.config_list($+1)= struct('config',new_config,'node_nb',node_nb+1);
    STANCE_LIST_ELMT_out.hcost = STANCE_LIST_ELMT_out.hcost+1;
    
    mprintf("Configuration added\n");
    mprintf("Connecting configuration to milestones\n");
    i_mile = 0;
    milestones =  LSH_query(STANCE_LIST_ELMT_out.roadmap.hash_LSH,PARAMS.metric,new_config,PARAMS.neigh_nb,PARAMS.PRM_radius);
    if ~isempty(milestones) then
        for i_mile = 1:length(milestones)
            [local_path, succ_path] = localPlanner(STANCE_LIST_ELMT_out.roadmap.config_list(milestones(i_mile)),new_config,50,PARAMS,STANCE_LIST_ELMT_out.stance,stance_type);
            if succ_path then
                STANCE_LIST_ELMT_out.roadmap.meta_graph = add_edge(milestones(i_mile),node_nb+1,STANCE_LIST_ELMT_out.roadmap.meta_graph);
                STANCE_LIST_ELMT_out.roadmap.edge_list($+1) = local_path;
            end
        end
    end
    
    STANCE_LIST_ELMT_out.hcost = STANCE_LIST_ELMT_out.hcost+i_mile/5;
    
    STANCE_LIST_ELMT_out.roadmap.hash_LSH = LSH_build(new_config,node_nb+1,STANCE_LIST_ELMT_out.roadmap.hash_LSH);
    
endfunction
