function [new_queue,element] = pop_head(old_queue)
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
        element = new_queue(1);
        new_queue(1)=null();
    else
        return;
    end
    
endfunction
