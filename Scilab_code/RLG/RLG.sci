function [P,O,THET,SUCCESS] = RLG(STANCE,NORMALS,PARAMS)
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
    //              *PARAMS.kpxy;
    //              *PARAMS.kpz;
    //              *PARAMS.kpRx;
    //              *PARAMS.kpRy;
    //              *PARAMS.tInc;
    
    //OUTPUT
    //
    
//----------------------------------------------------------------------------//
    P = 0;O = 0;THET = 0;SUCCESS = %F;
    
    //Compute LS-fit plane by ACP
    [footPlane_z,footPlane_d,footPlane_or] = plane_ACP(STANCE);
    footPlane_z = footPlane_z/norm(footPlane_z)
    
    //Project x0 on the plane, to define the plane frame
    if norm(cross(footPlane_z,[1 0 0]))<1.e-3 then //if x is perpendicular to plane
        footPlane_x = projectionPlan([0 1 0],footPlane_or,footPlane_z);
    else
        footPlane_x = projectionPlan([1 0 0],footPlane_or,footPlane_z);
    end
    footPlane_x = footPlane_x - footPlane_or;
    footPlane_x = footPlane_x/norm(footPlane_x);
    
    footPlane_y = cross(footPlane_z,footPlane_x);
    
    footPlane_Rmat = [footPlane_x;footPlane_y;footPlane_z];
    
    //Compute leg approximate workspaces. Project them on footPlane.
    foot_nb = size(STANCE,1);
    for i = 1:foot_nb
        //leg workspace, all points in R0
        WSmi_R0 = [];
        WSmi_proj_RP = [];
        //shell descriptions
        shellDesc_i = struct('origin',STANCE(i,:),'extRad',PARAMS.extRad(i),'intRad',PARAMS.intRad(i),'axis',NORMALS(i,:),'halfAngle',PARAMS.halfAngle);
        shellDesc(i) = shellDesc_i;
        
        WSmi_alpha = linspace(0,2*%pi,PARAMS.shellPtsNb);
        WSmi_theta = linspace(0,shellDesc_i.halfAngle,PARAMS.shellPtsNb);
        
        [x1,y1,z1] = halfSph(shellDesc_i.origin,shellDesc_i.extRad,WSmi_alpha,WSmi_theta,shellDesc_i.axis);
        WSmi_R0 = [x1',y1',z1'];
        
        if WSmi_intRadius then
            [x2,y2,z2] = halfSph(shellDesc_i.origin,shellDesc_i.intRad,WSmi_alpha,WSmi_theta,shellDesc_i.axis);
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
    Cxy = computeCxy(WS_proj_RP,[1 0;0 1]);
    if isnan(Cxy.origin) then
        disp('Could not compute intersection of workspaces!');
        return;
    end
    
    //Sample pxy_RP, transform into pxy_R0
    kpxy = 0;
    while kpxy<PARAMS.kpxy
        kpxy = kpxy+1;
        kpz = 0;
        pxy_RP = sampleInBBox(Cxy);
        pxy_R0 = footPlane_Rmat'*[pxy_RP 0]'+footPlane_or';
        mprintf("XY - At iteration %d of %d:\nBase xy position: [%.2f, %.2f]",kpxy,PARAMS.kpxy,pxy_R0(1),pxy_R0(2));
        
        //Compute intersections of the line perpendicular to footPlane, going through pxy_R0, with the WSmi
        line_z = struct('origin',pxy_R0,'direction',footPlane_z);
        for i=1:length(shellDesc)
            [bool,zInterval_i]=intersectLineWS
        
    end
    
endfunction
