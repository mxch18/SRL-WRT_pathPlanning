function distance = distanceBtwConfig(config1,config2)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //Outputs the distance between two configurations in R(3) + SO(3)
    
    //INPUT
    //config1: the first configuration
    //          config(1:3): position in R(3);
    //          config(4:7): quaternion defining the rotation in SO(3);
    //config2: the second configuration
    //          idem
    
    //OUTPUT
    //distance: the weighted distance according to the employed metrics
//----------------------------------------------------------------------------//
    R3_weight = 3;
    SO3_weight = 40;
    
    distance = R3_weight*norm(config1(1:3)-config2(1:3))+..
               SO3_weight*distQuat(config1(4:7),config2(4:7));
    
endfunction
