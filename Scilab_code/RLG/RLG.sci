function [Cxy,WS_proj_R0,footPlane_Rmat,zInterval,footPlane_x,footPlane_y] = RLG(STANCE,NORMALS,PARAMS)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //[P,O,THET,SUCCESS]
    
    //INPUT
    //STANCE: Row array of the current footholds. Contains struct describing
    //        footholds:
    //              *foothold: struct.
    //                  *foothold.leg: string identifying the leg (FR,FL,HR,HL);
    //                  *foothold.pos: row vector. Position of the foot in R0
    //PARAMS: a struct containing all the parameters relating to the robot 
    //        geometry, as well as problem-specific parameters:
    //              *PARAMS.extRad;
    //              *PARAMS.intRad;
    //              *PARAMS.halfAngle;
    //              *PARAMS.shellPtsNb;
    //              *PARAMS.shrink
    //              *PARAMS.kpxy;
    //              *PARAMS.kpz;
    //              *PARAMS.kRx;
    //              *PARAMS.kRy;
    //              *PARAMS.tInc;
    //              *PARAMS.aInc;
    //              *PARAMS.baseDimensions: (1) on x, (2) on y;
    
    //OUTPUT
    //
    
    //TODO : put angle range finding in function
    
//----------------------------------------------------------------------------//
    P = 0;O = 0;THET = 0;SUCCESS = %F;
    
    //Compute LS-fit plane by ACP
    stance_pos_list = STANCE(:).pos;
    stance_pos_array = [];
    for i=1:size(STANCE,2)
        stance_pos_array(i,:) = stance_pos_list(i);
    end
    
    [footPlane_z,footPlane_d,footPlane_or] = plane_ACP(stance_pos_array);
    footPlane_z = footPlane_z/norm(footPlane_z)
    
    //Project x0 on the plane, to define the plane frame
    if norm(cross(footPlane_z,[1 0 0]))<1.e-3 then //if x is perpendicular to plane
        disp('x perpendicular to plane, projecting y instead')
        footPlane_x = projectionPlan([0 1 0],footPlane_or,footPlane_z);
    else
        footPlane_x = projectionPlan([1 0 0],footPlane_or,footPlane_z);
    end
    footPlane_x = footPlane_x - footPlane_or;
    footPlane_x = footPlane_x/norm(footPlane_x);
    
//    footPlane_x=[1 0 0];
    
    footPlane_y = cross(footPlane_z,footPlane_x);
    
    footPlane_Rmat = [footPlane_x;footPlane_y;footPlane_z];
    
    //Compute leg approximate workspaces. Project them on footPlane.
    foot_nb = size(stance_pos_array,1);
    baseDiag = sqrt(PARAMS.baseDimensions(1)**2+PARAMS.baseDimensions(2)**2);
    tic()
    for i = 1:foot_nb
        //leg workspace, all points in R0
        WSmi_R0 = [];
        WSmi_proj_RP = [];
        //shell descriptions
        shellDesc_i = struct('origin',stance_pos_array(i,:),'extRad',PARAMS.extRad(i),'intRad',PARAMS.intRad(i),'axis',NORMALS(i,:),'halfAngle',PARAMS.halfAngle);
        shellDesc(i) = shellDesc_i;
        
        WSmi_alpha = linspace(0,2*%pi,PARAMS.shellPtsNb);
        WSmi_theta = linspace(0,shellDesc_i.halfAngle,PARAMS.shellPtsNb);
        
        [x1,y1,z1] = halfSph(shellDesc_i.origin,shellDesc_i.extRad,WSmi_alpha,WSmi_theta,shellDesc_i.axis);
        WSmi_R0 = [x1',y1',z1'];
        
        if shellDesc_i.intRad then
            [x2,y2,z2] = halfSph(shellDesc_i.origin,shellDesc_i.intRad,WSmi_alpha,WSmi_theta,shellDesc_i.axis);
            WSmi_R0 = [WSmi_R0;x2' y2' z2'];
        end
        WS_R0(:,:,i) = WSmi_R0;
        
        //projection, all points in RP
        for j=1:size(WSmi_R0,1)
            v = projectionPlan(WSmi_R0(j,:),footPlane_or,footPlane_z);
            WSmi_proj_R0(j,1) = v(1);WSmi_proj_R0(j,2) = v(2);WSmi_proj_R0(j,3) = v(3);
            v = footPlane_Rmat*(v'-footPlane_or');
            WSmi_proj_RP(j,1) = v(1);WSmi_proj_RP(j,2) = v(2);
        end
        WS_proj_RP(:,:,i) = WSmi_proj_RP;
        WS_proj_R0(:,:,i) = WSmi_proj_R0;
    end
    disp(toc())
    
    //Compute Cxy
    tic()
    Cxy = computeCxy(WS_proj_RP,[1 0;0 1]);
    if isnan(Cxy.origin) then
        mprintf('Could not compute intersection of workspaces! Stance is probably unfit\n');
        return;
    end
    disp(toc())
    //Sample pxy_RP, transform into pxy_R0
    kpxy = 0;
    while kpxy<PARAMS.kpxy
        kpxy = kpxy+1;
        kpz = 0;
        pxy_RP = sampleInBBox(Cxy,PARAMS.shrink);
        pxy_R0 = footPlane_Rmat'*[pxy_RP 0]'+footPlane_or';
//        mprintf("XY - At iteration %d of %d:\nBase xy_RP position: [%.4f, %.4f]\n",kpxy,PARAMS.kpxy,pxy_RP(1),pxy_RP(2));
        mprintf("XY - At iteration %d of %d:\nBase xy_R0 position: [%.4f, %.4f]\n",kpxy,PARAMS.kpxy,pxy_R0(1),pxy_R0(2));
        
        //Compute intersections of the line perpendicular to footPlane, going through pxy_R0, with the WSmi
        line_z = struct('origin',pxy_R0','direction',footPlane_z);
        for i=1:size(WS_R0,3)
            [boolInterZ_i,zInterval_i]=intersectLineWS(WS_R0(:,:,i),shellDesc(i),line_z,PARAMS.tInc);
//            boolInterZ(i) = boolInterZ_i;
            zInterval(i,:) = zInterval_i;
            if boolInterZ_i then
                mprintf("   Z - For leg %d, z range is: %.4f to %.4f\n",i,zInterval(i,1),zInterval(i,2));
            else
                mprintf("   Z - No intersection with leg %d workspace! Resampling pxy_RP\n",i);
                break;
            end
        end
        
        if ~boolInterZ_i then continue; end
        
        //Sample pz_R0
        zFinalInterval = min(zInterval,'r');
        while kpz<PARAMS.kpz
            kpz = kpz+1;
            kRx = 0;
            pz_R0 = zFinalInterval(1)+rand()*(zFinalInterval(2)-zFinalInterval(1));
            mprintf('Z - At iteration %d of %d:\n    Base z_R0 position: %.4f\n",kpz,PARAMS.kpz,pz_R0);
            
            //Compute intersections of Api arcs and WSmi for each rotation parameters
            //Rotations are represented by Euler angles (norm ZXZ): (psi,Z0);(thet,X1);(phi,Z2)
            base_R0 = [pxy_R0(1:2)', pz_R0];
            offset_i = [];
            xOff = [1 0 0]*PARAMS.baseDimensions(1);
            yOff = [0 1 0]*PARAMS.baseDimensions(2);
            T_EF_0 = eye(3,3); //Transformation matrix between end eff frame and R0. At first is identity.
            psiInter=cell(size(WS_R0,3));
            arcDesc_psi = struct('origin',base_R0,'normal',[0 0 1])
            //First start with (psi,Z0)
            for i=1:size(WS_R0,3)
                select STANCE(i).leg
                    case 'FR' then
                        offset_i = xOff + yOff;
                    case 'FL' then
                        offset_i = - xOff + yOff;
                    case 'HR' then
                        offset_i= + xOff - yOff;
                    case 'HL' then
                        offset_i = - xOff - yOff;
                    else
                        mprintf("Error in the definition of foothold %d : leg name does not exist!",i);
                end
                [boolInterPsi_i,psiMultiple_i,psiInter_i] = intersectArcWS(WS_R0(:,:,i),offset_i,T_EF_0,shellDesc(i),arcDesc_psi,PARAMS.aInc);
                if boolInterPsi_i then
                    psiInter(i).entries = createInterval(psiInter_i);
                    if psiMultiple_i then
                        mprintf("   PSI - For leg %d, psi lies in %d different intervals", size(psiInter(i).entries,1));
                    else
                        mprintf("   PSI - For leg %d, psi range is: %.4f to %.4f\n",i,psiInter(i).entries(1),psiInter(i).entries(2));
                    end
                else
                    mprintf("   PSI - No intersection with leg %d workspace! Resampling pz_0\n",i);
                    break;
                end
            end
            if ~boolInterPsi_i then continue; end
            
        end
        
    end
    
endfunction
