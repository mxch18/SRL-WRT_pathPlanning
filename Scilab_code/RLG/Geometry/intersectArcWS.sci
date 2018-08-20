function [bool,multiple,angleRange] = intersectArcWS(WSmi_R0,offset,T_mat,shellDesc,arcDesc,aInc)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //
    
    //INPUT
    //WSmi_R0: the workspace approximation in R0;
    //offset : the point to which the kinematic chain is attached to the end 
    //        effector, in the end effector frame;
    //Tmat: the transformation matrix between EF frame and R0;
    //shellDesc: a struct containing the shell parameters
    //              *shellDesc.origin
    //              *shellDesc.extRad
    //              *shellDesc.intRad
    //              *shellDesc.axis
    //              *shellDesc.halfAngle
    //arcDesc: struct. The description of the arc:
    //              *arcDesc.center: arc center
    //              *arcDesc.normal: arc normal (axis of rotation)
    //aInc: increment on the arc exploration. Radians
    
    //OUTPUT
    //
    
//----------------------------------------------------------------------------//
    bool = %F;
    multiple = %F;
    angleRange = [];
    
    //Pre-compute part of rotation matrix
    arcDesc.normal = arcDesc.normal/norm(arcDesc.normal);
    ux = arcDesc.normal(1);
    uy = arcDesc.normal(2);
    uz = arcDesc.normal(3);
    P = [ux**2 ux*uy ux*uz;ux*uy uy**2 uy*uz;ux*uz uy*uz uz**2];
    I = eye(3,3);
    Q = [0 -uz uy;uz 0 -ux;-uy ux 0];
    
    angle_rot = -%pi;
    
//    angleValid = [];
//    angleNotValid = [];
    k=1;
    n=1;
    
    c = cos(angle_rot);
    s = sin(angle_rot);
    R_mat = P+c*(I-P)+s*Q;
    Api_R0 = arcDesc.origin' + T_mat*R_mat*offset';
    
    if isInShell(shellDesc,Api_R0') then
        bool = %T;
        boolLast = %T;
        boolNow = %T;
        angleRange(k) = angle_rot;
        k=k+1;
    else
        boolLast = %F;
        boolNow = %F;
//        angleNotValid(1) = angle_rot;
    end
    
    while angle_rot<%pi
        angle_rot = angle_rot + aInc;
        c = cos(angle_rot);
        s = sin(angle_rot);
//        R_mat = [ux**2*(1-c)+c ux*uy*(1-c)-uz*s ux*uz*(1-c)+uy*s;ux*uy*(1-c)+uz*s uy**2*(1-c)+c uy*uz*(1-c)-ux*s;ux*uz*(1-c)-uy*s uy*uz*(1-c)+ux*s uz**2*(1-c)+c];
        R_mat = P+c*(I-P)+s*Q;
        Api_R0 = arcDesc.origin' + T_mat*R_mat*offset'; //the attachment point in R0
        if isInShell(shellDesc,Api_R0') then
            bool = %T;
            boolLast = boolNow;
            boolNow = %T;
            if (~boolLast&boolNow)|(boolLast&~boolNow) then
                angleRange(k) = angle_rot;
                k = k+1;
            end
        else
            boolLast = boolNow;
            boolNow = %F;
            if (~boolLast&boolNow)|(boolLast&~boolNow) then
                angleRange(k) = angle_rot-aInc;
                k = k+1;
            end
        end
    end
    
    if length(angleRange)>2 then
        multiple = %T;
    end
    
endfunction
