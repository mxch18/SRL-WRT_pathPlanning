function [out,stackOut] = stackPop(stackIn)
    //get the last inserted element in the stack and removes it from the stack
    out = stackIn(1);
    stackIn(1) = null();
    stackOut = stackIn;
endfunction
