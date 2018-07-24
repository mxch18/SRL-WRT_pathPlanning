function outMat = removeRowFromMat(inMat,row)
    //Removes a row from a matrix
    //
    pos = members(inMat, row, "rows");
    pos = find(pos~=0,1); //only removes first occurence of line
    outMat = [inMat(1:pos-1,:);inMat(pos+1:$,:)];
endfunction
