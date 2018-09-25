function element = peep_member(old_queue,index)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //
    
    //INPUT
    //old_queue; a priority queue based on the cost. No limit on size
    //new_element: struct:
    //          *new_element.node_cost
    //          *new_element.node_number
    
    //OUTPUT
    //
    
//----------------------------------------------------------------------------//
    new_queue = old_queue;
    element = 0;
    
    if length(new_queue)>0 then
        element = new_queue(index);
    else
        return;
    end
    
endfunction
