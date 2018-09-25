//test graph creation
clear;clc;close();getd("./Graph_search");getd("./kNN");getd(".");getd("./Planner")

function_set = struct('query_ws',in_ball_naive,'classifier',classify_pts);

f1=struct('leg','HL','pos',[-0.3,-0.3,0]);
f2=struct('leg','FL','pos',[-0.3,+0.3,0]);
f3=struct('leg','HR','pos',[+0.3,-0.3,0]);
f4=struct('leg','FR','pos',[+0.3,+0.3,0]);

stance = [f1,f2,f3,f4];
//
//for i=1:size(stance,2)
//    stance_pos(i,:) = stance.pos(i);
//end
//
//// Fake dataset
//dataset = -1+2*rand(200,3);
//
//pts_reach = in_ball_naive(stance,dataset,params_prn);
//
//pts_classified = classify_pts(pts_reach,stance);
//
//new_stances = find_adjacent(stance,dataset,params_prn,function_set);

//scatter3(dataset(:,1),dataset(:,2),dataset(:,3));
//scatter3(stance_pos(:,1),stance_pos(:,2),stance_pos(:,3),'red');
//scatter3(pts_reach(:,1),pts_reach(:,2),pts_reach(:,3),'green');

// Real dataset

filen = mopen("./points.txt","r");
p = zeros(1,3)
i = 1

while ~meof(filen) do
    p(i,1:3) = mfscanf(1,filen,"%f %f %f");
    i = i+1;
end

mclose(filen);

i1=struct('leg','HL','pos',[-2.3344e-1, -4.9237e+0, -4.7664e-1]);
i2=struct('leg','FL','pos',[-2.3184e-1, -4.0805e+0, -4.6890e-1]);
i3=struct('leg','HR','pos',[+2.3070e-1, -4.9226e+0, -4.8433e-1]);
i4=struct('leg','FR','pos',[+2.3249e-1, -4.0774e+0, -4.6157e-1]);

stance_ini = [i1,i2,i3,i4];

for i=1:size(stance_ini,2)
    stance_pos(i,:) = stance_ini.pos(i);
end

ref_pt = min(p,'r');
cell_size = 0.5;
params_prn = struct('ball_radius',0.6,'origin',ref_pt,'cell_size',cell_size,'goal_stance',stance,'function_set',function_set,'hash_size',1013,'nb',5,'ne',1);


pts_reach = in_ball_naive(stance_ini,p,params_prn);

pts_classified = classify_pts(pts_reach,stance_ini);

new_stances = find_adjacent(stance_ini,p,params_prn);

metanet_graph = make_graph('stance_graph',1,1,[1],[1]);
metanet_graph = delete_edges([1 1],metanet_graph);

stance_list = list(struct('stance',stance_ini,'gcost',0,'hcost',0));

stance_hash = list();
stance_hash(1014) = 0;
stance_hash(1014) = null();

centroid = mean(stance_pos,'r'); //centroid of the initial stance
hash_bin = hash_XOR(centroid,length(stance_hash),cell_size,ref_pt);
stance_hash(hash_bin) = [centroid, -1];

edge_list = list();

cost_list = list(struct('node_number',1,'node_cost',stance_list(1).hcost,'expanded',%T));

// [bool_add,bool_fin,stance_graph_out] = add_stance_to_graph(STNC,parent_node_nb,stance_graph,params)

stance_graph = struct('metanet_graph',metanet_graph,'stance_list',stance_list,'edge_list',edge_list,'stance_hash',stance_hash,'cost_list',cost_list)

//[b_add,b_fin,stance_graph] = add_stance_to_graph(new_stances(1,:),1,stance_graph,params_prn);

[bool_success,path,expansion_nodes,single_cndts_node_nb] = plan(stance_ini,p,params_prn)

