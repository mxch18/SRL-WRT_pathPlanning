function [out,stackOut] = stackPop(stackIn)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //Get the last inserted element in the stack and removes it from the stack (FIFO)
    
    //INPUT:
    //stackIn: the current stack
    
    //OUTPUT:
    //out: the removed element
    //stackOut: the updated stack
    
    //TODO: Handle empty stack
    
//----------------------------------------------------------------------------//

    out = stackIn(1);
    stackIn(1) = null();
    stackOut = stackIn;
endfunction
