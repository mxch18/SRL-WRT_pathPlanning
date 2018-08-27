function [P,O,THETA,RMAT,SUCCESS] = RLG(STANCE,NORMALS,PARAMS)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
//    Cxy,WS_proj_R0,footPlane_Rmat,zFinalInterval,psiInter,thetInter,phiInter
    //[]
    
    //INPUT
    //STANCE: Row array of the current footholds. Contains struct describing
    //        footholds:
    //              *foothold: struct.
    //                  *foothold.leg: string identifying the leg (FR,FL,HR,HL);
    //                  *foothold.pos: row vector. Position of the foot in R0
    //PARAMS: a struct containing all the parameters relating to the robot 
    //        geometry, as well as problem-specific parameters:
    //              *PARAMS.extRad;
    //              *PARAMS.distApiOb: distances between the leg attachment and the EoF CoM
    //              *PARAMS.intRad;
    //              *PARAMS.halfAngle;
    //              *PARAMS.shellPtsNb;
    //              *PARAMS.shrink
    //              *PARAMS.kpxy;
    //              *PARAMS.kpz;
    //              *PARAMS.kRz;
    //              *PARAMS.kRx;
    //              *PARAMS.tInc;
    //              *PARAMS.aInc;
    //              *PARAMS.baseDimensions: (1) on x, (2) on y;
    //              *PARAMS.legLength: [l1,l2,l3]
    //              *PARAMS.verbose: %T or %F
    
    //OUTPUT
    //
    
    //TODO : put psi/theta/phi range finding in function
    //       !!!!!!!!!!!!!!!!!!! ADD AUGMENTED WORKSPACES !!!!!!!!!!!!!!!!!!!!
    //       put IK in function
    
//----------------------------------------------------------------------------//
    P = 0;O = 0;THETA = 0;SUCCESS = %F;RMAT = 0;
    
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
//    tic()
    for i = 1:foot_nb
        //leg workspace, all points in R0
        WSmi_R0 = [];
        WSmi_proj_RP = [];
        //shell descriptions
        
        shellDesc_i = struct('origin',stance_pos_array(i,:),'extRad',PARAMS.extRad(i),'intRad',PARAMS.intRad(i),'axis',NORMALS(i,:),'halfAngle',PARAMS.halfAngle);
        shellDesc(i) = shellDesc_i;
        
        shellDesc_AUG_i = struct('origin',stance_pos_array(i,:),'extRad',PARAMS.extRad(i)+PARAMS.distApiOb(i),'intRad',PARAMS.intRad(i),'axis',NORMALS(i,:),'halfAngle',PARAMS.halfAngle);
        shellDesc_AUG(i) = shellDesc_AUG_i;
        
        WSmi_alpha = linspace(0,2*%pi,PARAMS.shellPtsNb);
        WSmi_theta = linspace(%pi/2-shellDesc_AUG_i.halfAngle,%pi/2,PARAMS.shellPtsNb);
        
        [x1,y1,z1] = halfSph(shellDesc_AUG_i.origin,shellDesc_AUG_i.extRad,WSmi_alpha,WSmi_theta,shellDesc_AUG_i.axis);
        WSmi_R0 = [x1',y1',z1'];
        
        if shellDesc_i.intRad then
            [x2,y2,z2] = halfSph(shellDesc_AUG_i.origin,shellDesc_AUG_i.intRad,WSmi_alpha,WSmi_theta,shellDesc_AUG_i.axis);
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
//    disp(toc())
    
    //Compute Cxy
    tic()
    Cxy = computeCxy(WS_proj_RP,[1 0;0 1]);
    if isnan(Cxy.origin) then
        if PARAMS.verbose then
            mprintf('Could not compute intersection of workspaces! Stance is probably unreachable...\n');
        end
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
        if PARAMS.verbose then
            mprintf("XY - At iteration %d of %d:\nBase xy_R0 position: [%.4f, %.4f]\n",kpxy,PARAMS.kpxy,pxy_R0(1),pxy_R0(2));
        end
        
        zInterval = cell(1,foot_nb);
        //Compute intersections of the line perpendicular to footPlane, going through pxy_R0, with the WSmi
        line_z = struct('origin',pxy_R0','direction',footPlane_z);
        for i=1:foot_nb
            [boolInterZ_i,zMultiple_i,zInterval_i,d_i]=intersectLineWS(WS_R0(:,:,i),shellDesc_AUG(i),line_z,PARAMS.tInc);
//            boolInterZ(i) = boolInterZ_i;
            if boolInterZ_i then
                zInterval(i).entries = createZInterval(zInterval_i,d_i);
                if PARAMS.verbose & zMultiple_i then
                    mprintf("   Z - For leg %d, z lies in %d different intervals", i, size(psiInter(i).entries,1));
                elseif PARAMS.verbose then
                    mprintf("   Z - For leg %d, z range is: %.4f to %.4f\n",i,zInterval(i).entries(1),zInterval(i).entries(2));
                end
            else
                if PARAMS.verbose then
                    mprintf("   Z - No intersection with leg %d workspace! Resampling pxy_RP...\n",i);
                end
                break;
            end
        end
        
        if ~boolInterZ_i then continue; end
        
        //Sample pz_R0
        [zFinalBool,zFinalInterval] = intersectSetIntervals(zInterval);
        if ~zFinalBool then
            if PARAMS.verbose then
                mprintf("   Z - z valid intervals do not intersect! Resammpling pxy_RP...\n");
            end
            continue; 
        end
        
        while kpz<PARAMS.kpz
            kpz = kpz+1;
            kRz = 0;
            pz_R0 = sampleFromMultInterval(zFinalInterval);
            if PARAMS.verbose then
                mprintf('Z - At iteration %d of %d:\n   Base z_R0 position: %.4f\n",kpz,PARAMS.kpz,pz_R0);
            end
            
            //Compute intersections of Api arcs and WSmi for each rotation parameters
            //Rotations are represented by Euler angles (norm ZXY): (psi,Z0);(thet,X1);(phi,Y2)
            base_R0 = [pxy_R0(1:2)', pz_R0];
            
            P = base_R0;
            
            offset_i = [];
            xOff = [1 0 0]*PARAMS.baseDimensions(1)/2;
            yOff = [0 1 0]*PARAMS.baseDimensions(2)/2;
            R_0_EF = eye(3,3); //Transformation matrix between end-eff frame and R0. At first is identity.
            psiInter=cell(1,foot_nb);
            arcDesc_psi = struct('origin',base_R0,'normal',[0 0 1]) //rotation around Z0
            //First start with (psi,Z0)
            for i=1:foot_nb
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
                        if PARAMS.verbose then
                            mprintf("Error in the definition of foothold %d : leg name does not exist!\n",i);
                        end
                        return;
                end
                [boolInterPsi_i,psiMultiple_i,psiInter_i] = intersectArcWS(WS_R0(:,:,i),offset_i,R_0_EF,shellDesc(i),arcDesc_psi,PARAMS.aInc);
                if boolInterPsi_i then
                    psiInter(i).entries = createAngleInterval(psiInter_i);
                    if PARAMS.verbose & psiMultiple_i then
                        mprintf("   PSI - For leg %d, psi lies in %d different intervals\n", i, size(psiInter(i).entries,1));
                    elseif PARAMS.verbose then
                        mprintf("   PSI - For leg %d, psi range is: %.4f to %.4f\n",i,psiInter(i).entries(1),psiInter(i).entries(2));
                    end
                else
                    if PARAMS.verbose then
                        mprintf("   PSI - No intersection with leg %d workspace! Resampling pz_0...\n",i);
                    end
                    break;
                end
            end
            if ~boolInterPsi_i then continue; end
            
            //Sample (psi,Z0)
            [psiBoolFinal,psiFinalInterval] = intersectSetIntervals(psiInter);
            if ~psiBoolFinal then
                if PARAMS.verbose then
                    mprintf("   PSI - psi valid intervals do not intersect! Resampling z_R0...\n");
                end
                continue;
            end
            
            while kRz<PARAMS.kRz
                kRz = kRz +1;
                kRx = 0;
                psi = sampleFromMultInterval(psiFinalInterval);
                if PARAMS.verbose then
                    mprintf("PSI - At iteration %d of %d:\n   Base psi: %.4f\n",kRz,PARAMS.kRz,psi);
                end
                
                //Rotate base
                Rz0 = [cos(psi), -sin(psi), 0;sin(psi), cos(psi) 0;0 0 1];
                R_0_EF = R_0_EF*Rz0;
                thetInter=cell(1,foot_nb);
                arcDesc_thet = struct('origin',base_R0,'normal',[1 0 0]) //rotation around X1
                //Then (thet,X1)
                for i=1:foot_nb
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
                            if PARAMS.verbose then
                                mprintf("Error in the definition of foothold %d : leg name does not exist!\n",i);
                            end
                            return;
                    end
                    [boolInterThet_i,thetMultiple_i,thetInter_i] = intersectArcWS(WS_R0(:,:,i),offset_i,R_0_EF,shellDesc(i),arcDesc_thet,PARAMS.aInc);
//                    disp(thetInter_i);
                    if boolInterThet_i then
                        thetInter(i).entries = createAngleInterval(thetInter_i);
                        if PARAMS.verbose & thetMultiple_i then
                            mprintf("   THET - For leg %d, theta lies in %d different intervals\n", i, size(thetInter(i).entries,1));
                        elseif PARAMS.verbose then
                            mprintf("   THET - For leg %d, theta range is: %.4f to %.4f\n",i,thetInter(i).entries(1),thetInter(i).entries(2));
                        end
                    else
                        if PARAMS.verbose then
                            mprintf("   THET - No intersection with leg %d workspace! Resampling psi...\n",i);
                        end
                        break;
                    end
                end
                if ~boolInterThet_i then continue; end
                
                //Sample (theta,X1)
               [thetBoolFinal,thetFinalInterval] = intersectSetIntervals(thetInter);
                if ~thetBoolFinal then
                    if PARAMS.verbose then
                        mprintf("   THET - theta valid intervals do not intersect! Resampling psi...\n");
                    end
                    continue;
                end
                
                while kRx<PARAMS.kRx
                    kRx = kRx +1;
                    theta = sampleFromMultInterval(thetFinalInterval);
                    if PARAMS.verbose then
                        mprintf("THETA - At iteration %d of %d:\n   Base theta: %.4f\n",kRx,PARAMS.kRx,theta);
                    end
                    //Rotate base
                    Rx1 = [1, 0, 0;0, cos(theta), -sin(theta);0 sin(theta) cos(theta)];
                    R_0_EF = R_0_EF*Rx1;
                    phiInter=cell(1,foot_nb);
                    arcDesc_phi = struct('origin',base_R0,'normal',[0 1 0]) //rotation around Y2
                    //Then (phi,Y2)
                    for i=1:foot_nb
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
                                if PARAMS.verbose then
                                    mprintf("Error in the definition of foothold %d : leg name does not exist!\n",i);
                                end
                                return;
                        end
                        [boolInterPhi_i,phiMultiple_i,phiInter_i] = intersectArcWS(WS_R0(:,:,i),offset_i,R_0_EF,shellDesc(i),arcDesc_phi,PARAMS.aInc);
                        if boolInterPhi_i then
                            phiInter(i).entries = createAngleInterval(phiInter_i);
                            if PARAMS.verbose & phiMultiple_i then
                                mprintf("   PHI - For leg %d, phi lies in %d different intervals\n", i, size(phiInter(i).entries,1));
                            elseif PARAMS.verbose then
                                mprintf("   PHI - For leg %d, phi range is: %.4f to %.4f\n",i,phiInter(i).entries(1),phiInter(i).entries(2));
                            end
                        else
                            if PARAMS.verbose then
                                mprintf("   PHI - No intersection with leg %d workspace! Resampling theta...\n",i);
                            end
                            break;
                        end
                    end
                    if ~boolInterPhi_i then continue; end
                    
                    //Sample (phi,Y2)
                    [phiBoolFinal,phiFinalInterval] = intersectSetIntervals(phiInter);
                    if ~phiBoolFinal then
                        if PARAMS.verbose then
                            mprintf("   PHI - phi valid intervals do not intersect! Resampling theta...\n");
                        end
                        continue;
                    end
                    
                    phi = sampleFromMultInterval(phiFinalInterval);
                    if PARAMS.verbose then
                        mprintf("PHI - Base phi: %.4f\n",phi);
                    end
                    
                    Ry2 = [cos(phi), 0, sin(phi);0, 1 0;-sin(phi) 0 cos(phi)];
                    R_0_EF = R_0_EF*Ry2;
                    
                    RMAT = R_0_EF;
                    
                    O = [psi,theta,phi];
                    
                    if PARAMS.verbose then
                        mprintf("\nBase state sampled! Now using closed form IK for the legs...\n");
                    end
                    
                    for i=1:foot_nb
                        select STANCE(i).leg
                            case 'FR' then
                                offset_i = xOff + yOff;
                                R_Leg_EF = [0 1 0;1 0 0;0 0 -1];
                                factor_t2 = -1;
                                factor_t3 = -1;
                                factor_elbow = +1;
                            case 'FL' then
                                offset_i = - xOff + yOff;
                                R_Leg_EF = [0 1 0;-1 0 0;0 0 1];
                                factor_t2 = +1;
                                factor_t3 = +1;
                                factor_elbow = -1;
                            case 'HR' then
                                offset_i= + xOff - yOff;
                                R_Leg_EF = [0 -1 0;1 0 0;0 0 1];
                                factor_t2 = +1;
                                factor_t3 = +1;
                                factor_elbow = -1;
                            case 'HL' then
                                offset_i = - xOff - yOff;
                                R_Leg_EF = [0 -1 0;-1 0 0;0 0 -1];
                                factor_t2 = -1;
                                factor_t3 = -1;
                                factor_elbow = +1;
                        end
                        
//                        disp(offset_i);
//                        disp(R_0_EF);
//                        disp(R_Leg_EF);
                        
                        IK_target_RLeg = -R_Leg_EF*offset_i' + R_Leg_EF*R_0_EF'*(STANCE(i).pos'-base_R0'); //the foothold for the ith leg, in the leg base frame
                        IK_target_array(:,i) = IK_target_RLeg;
//                        disp(IK_target_RLeg);
                        
                        THETA(i,1) = atan(IK_target_RLeg(2),IK_target_RLeg(1));
                        
                        rem = sqrt(IK_target_RLeg(1)**2+IK_target_RLeg(2)**2)-PARAMS.legLength(1);
                        nc3 = IK_target_RLeg(3)**2+rem**2-PARAMS.legLength(2)**2-PARAMS.legLength(3)**2;
                        dc3 = 2*PARAMS.legLength(2)*PARAMS.legLength(3);
                        c3 = nc3/dc3;
                        
                        if abs(c3)>1 then
//                            disp(c3)
                            if PARAMS.verbose then
                                mprintf("\nIK - NO SOLUTION FOR LEG %s INVERSE KINEMATICS\n",STANCE(i).leg);
                            end
                            return;
                        end
                        s3 = factor_elbow*sqrt(1-c3**2); //ELBOw UP
                        THETA(i,3) = factor_t3*atan(s3,c3);
                        
                        THETA(i,2) = factor_t2*(atan(IK_target_RLeg(3),rem)-atan(PARAMS.legLength(3)*s3,PARAMS.legLength(2)+PARAMS.legLength(3)*c3))
                    end
                    SUCCESS=%T;
                    return;
                end
                
                if PARAMS.verbose then
                    mprintf("THET - Reached maximum number of trials, resampling psi...\n");
                end
                
            end
            
            if PARAMS.verbose then
                mprintf("PSI - Reached maximum number of trials, resampling z...\n");
            end
            
        end
        
        if PARAMS.verbose then
            mprintf("Z - Reached maximum number of trials, resampling XY...\n");
        end
        
    end
    
    if PARAMS.verbose then
        mprintf("XY - Reached maximum number of trials, aborting...\n");
    end
    
endfunction
