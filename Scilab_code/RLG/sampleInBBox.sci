function point = sampleInBBox(bbox)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //
    
    //INPUT
    //bbox: the definition of the bounding box. struct.
    
    //OUTPUT
    //point: the point in the bounding box
    
//----------------------------------------------------------------------------//
    
    point = bbox.origin+bbox.length*bbox.v1+bbox.width*bbox.v2;
    
endfunction
