function distance = distanceBtwConfig(config1,config2,stance_type)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //Outputs the distance between two configurations in SE(3)
    
    //INPUT
    //config1: the first configuration
    //          config(1:3): position in R(3);
    //          config(4:7): quaternion defining the rotation in SO(3);
    //config2: the second configuration
    //          idem
    //stance_type: 3 or 4-stance
    
    //OUTPUT
    //distance: the weighted distance according to the employed metrics
//----------------------------------------------------------------------------//
    R3_weight = 30.6;
    SO3_weight = 17.1;
    Swing_weight = 6.4;
    
    distance = R3_weight*norm(config1(1:3)-config2(1:3))+..
               SO3_weight*distQuat(config1(4:7),config2(4:7));
    
    if stance_type == 3 then
        angles_1 = config1(8:10); angles_2 = config2(8:10);
        angles_diff = atan(sin(angles_1-angles_2),cos(angles_1-angles_2));
        distance = distance + Swing_weight*norm(angles_diff);
    else
    end
    
    
endfunction
