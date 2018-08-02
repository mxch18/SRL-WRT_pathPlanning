function outMat = removeRowFromMat(inMat,row)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //Removes a row from a matrix
    
    //INPUT
    
    //OUTPUT
    
//----------------------------------------------------------------------------/
    
    
    pos = members(inMat, row, "rows");
    pos = find(pos~=0,1); //only removes first occurence of line
    outMat = [inMat(1:pos-1,:);inMat(pos+1:$,:)];
endfunction
