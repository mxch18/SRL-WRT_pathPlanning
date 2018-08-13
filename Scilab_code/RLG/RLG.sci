function [P,O,THET] = RLG(STANCE,NORMALS,PARAMS)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //
    
    //INPUT
    //PARAMS: a struct containing all the parameters relating to the robot 
    //        geometry, as well as problem-specific parameters:
    //              *PARAMS.extRad;
    //              *PARAMS.intRad;
    //              *PARAMS.halfAngle;
    //              *PARAMS.shellPtsNb;
    
    //OUTPUT
    //
    
//----------------------------------------------------------------------------//
    
    
    //Compute LS-fit plane by ACP
    [footPlane_z,footPlane_d,footPlane_or] = plane_ACP(STANCE);
    //Project x0 on the plane, to define the plane base
    if norm(cross(footPlane_z,[1 0 0]))<1.e-3 then
        footPlane_x = projectionPlan([0 1 0],footPlane_or,footPlane_z);
    else
        footPlane_x = projectionPlan([1 0 0],footPlane_or,footPlane_z);
    end
    footPlane_y = cross(footPlane_z,footPlane_x);
    
    footPlane_Rmat = [footPlane_x;footPlane_y;footPlane_z];
    
    //Compute leg approximate workspaces. Project them on footPlane.
    foot_nb = size(STANCE,1);
    for i = 1:foot_nb
        //leg workspace, all points in R0
        WSmi_R0 = [];
        WSmi_proj_RP = [];
        
        WSmi_origin = STANCE(i,:);
        WSmi_extRadius = PARAMS.extRad(i);
        WSmi_intRadius = PARAMS.intRad(i);
        WSmi_alpha = linspace(0,2*%pi,PARAMS.shellPtsNb);
        WSmi_theta = linspace(0,PARAMS.halfAngle,PARAMS.shellPtsNb);
        WSmi_direction = NORMALS(i,:);
        [x1,y1,z1] = halfSph(WSmi_origin,WSmi_extRadius,WSmi_alpha,WSmi_theta,WSmi_direction);
        WSmi_R0 = [x1',y1',z1'];
        if WSmi_intRadius then
            [x2,y2,z2] = halfSph(WSmi_origin,WSmi_intRadius,WSmi_alpha,WSmi_theta,WSmi_direction);
            WSmi_R0 = [WSmi_R0;x2' y2' z2'];
        end
        WS_R0(:,:,i) = WSmi_R0;
        
        //projection, all points in RP
        for j=1:size(WSmi_R0,1)
            v = projectionPlan(WSmi_R0(j,:),footPlane_or,footPlane_z);
            v = footPlane_Rmat*v';
            WSmi_proj_RP(j,1) = v(1);WSmi_proj_RP(j,2) = v(2);
        end
        WS_proj_RP(:,:,i) = WSmi_proj_RP;
    end
    
    //Compute Cxy
    
    
endfunction
