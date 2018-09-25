function new_queue = insert_in_queue(old_queue,new_element)
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
    
    for i=length(new_queue):-1:1
        if new_element.node_cost>=new_queue(i).node_cost then
            new_queue(i+1) = new_element;
            break;
        else
            new_queue(i+1) = new_queue(i);
            new_queue(i) = new_element;
        end
    end
    
endfunction
