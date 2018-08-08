function stackOut = stackInsert(stackIn, in)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //Insert an element into a stack in a FIFO way.
    
    //INPUT:
    //stackIn: the current stack
    //in: the element we want to add in first position
    
    //OUTPUT:
    //stackOut: the updated stack
    
    //TODO: Handle wrong "in" type
    
//----------------------------------------------------------------------------//
    
    stackIn(0)=in;
    stackOut = stackIn;
endfunction
