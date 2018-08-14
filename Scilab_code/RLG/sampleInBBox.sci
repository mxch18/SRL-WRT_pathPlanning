function point = sampleInBBox(bbox,shrink)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //
    
    //INPUT
    //bbox: the definition of the bounding box. struct.
    
    //OUTPUT
    //point: the point in the bounding box
    
//----------------------------------------------------------------------------//
    newOri = bbox.origin+(1-shrink)/2*(bbox.length*bbox.v1+bbox.width*bbox.v2);
    point = newOri+shrink*(rand()*bbox.length*bbox.v1+rand()*bbox.width*bbox.v2);
    
endfunction
