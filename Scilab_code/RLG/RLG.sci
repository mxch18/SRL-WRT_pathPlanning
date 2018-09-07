function [P,Q,THETA,RMAT,SUCCESS,footPlane_Q,footPlane_Rmat] = RLG(STANCE,NORMALS,PARAMS)
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
    //              *PARAMS.kRot
    //              *PARAMS.tInc;
    //              *PARAMS.aInc;
    //              *PARAMS.baseDimensions: (1) on x, (2) on y;
    //              *PARAMS.legLength: [l1,l2,l3]
    //              *PARAMS.verbose: %T or %F
    
    //OUTPUT
    //P: the base position
    //Q: the quaternion defining the rotation
    //THETA: the leg's joint angles
    //RMAT: the rotation matrix
    //SUCCESS: boolean for benchmarking purposes (atm)
    
    //TODO : put psi/theta/phi range finding in function
    //       could you put bounds on phi so that it doesn't flip over? NAH(?)
    //       put IK in function
    //       Remove RMAT output
    //       Change rotation parametrization to full quaternion
    
//----------------------------------------------------------------------------//
    P = 0;Q = 0;THETA = 0;SUCCESS = %F;RMAT = 0;
    
    //Compute LS-fit plane by ACP
    stance_pos_list = STANCE(:).pos;
    stance_pos_array = [];
    for i=1:size(STANCE,2)
        stance_pos_array(i,:) = stance_pos_list(i);
    end
    
    foot_nb = size(stance_pos_array,1);
    
    [footPlane_z,footPlane_d,footPlane_or] = plane_ACP(stance_pos_array);
    footPlane_z = footPlane_z/norm(footPlane_z);
    

    FL_present = %f;HL_present = %f;FR_present = %f;HR_present = %f;
    for i=1:foot_nb
        select STANCE(i).leg
            case 'FL' then
                FL = i;
                FL_present = %t;
            case 'HL' then
                HL = i;
                HL_present = %t;
            case 'FR' then
                FR = i;
                FR_present = %t;
            case 'HR' then
                HR = i;
                HR_present = %t;
        end
    end
    
    if HR_present&FR_present then
//        footPlane_x = (STANCE(HR).pos-footPlane_or) + 0.5*(STANCE(FR).pos-STANCE(HR).pos);
        footPlane_x = projectionDroite(footPlane_or,STANCE(HR).pos,STANCE(FR).pos-STANCE(HR).pos); //vector from HR to orth proj of stance centroid on line HR-FR
        footPlane_x = footPlane_x - footPlane_or;
        footPlane_x = footPlane_x;
//        footPlane_x = projectionPlan(footPlane_x,footPlane_or,footPlane_z);
//        footPlane_x = footPlane_x - (footPlane_z*footPlane_or')*footPlane_z
        footPlane_x = footPlane_x/norm(footPlane_x);
        
        footPlane_y = cross(footPlane_z,footPlane_x);
        
    elseif HL_present&FL_present then
//        footPlane_x = (STANCE(HL).pos-footPlane_or) + 0.5*(STANCE(FL).pos-STANCE(HL).pos);
        footPlane_x = projectionDroite(footPlane_or,STANCE(HL).pos,STANCE(FL).pos-STANCE(HL).pos); //vector from HL to orth proj of stance centroid on line HL-FL
        footPlane_x = footPlane_x - footPlane_or;
        footPlane_x = -footPlane_x;
//        disp(footPlane_x)
//        footPlane_x = projectionPlan(footPlane_x,footPlane_or,footPlane_z);
//        footPlane_x = footPlane_x - (footPlane_z*footPlane_or')*footPlane_z
//        disp(footPlane_x)
        footPlane_x = footPlane_x/norm(footPlane_x);
        
        footPlane_y = cross(footPlane_z,footPlane_x);
    end
    
    footPlane_Rmat = [footPlane_x;footPlane_y;footPlane_z]; //R_P_0
    [footPlane_angle,footPlane_vector] = angle_vector_FromMat(footPlane_Rmat);
    footPlane_Q = createQuaternion(footPlane_angle,footPlane_vector);
    
    //Compute leg approximate workspaces. Project them on footPlane.
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
        
        [x1,y1,z1] = halfSph(shellDesc_AUG_i.origin,shellDesc_AUG_i.extRad,2*WSmi_alpha,WSmi_theta,shellDesc_AUG_i.axis);
        WSmi_R0 = [x1',y1',z1'];
        
        if shellDesc_i.intRad then
            [x2,y2,z2] = halfSph(shellDesc_AUG_i.origin,shellDesc_AUG_i.intRad,2*WSmi_alpha,WSmi_theta,shellDesc_AUG_i.axis);
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
    
    //Compute Cxy
    Cxy = computeCxy(WS_proj_RP,[1 0;0 1]);
    if isnan(Cxy.origin) then
        if PARAMS.verbose then
            mprintf('Could not compute intersection of workspaces! Stance is probably unreachable...\n');
        end
        return;
    end
    //Sample pxy_RP, transform into pxy_R0
    kpxy = 0;
    while kpxy<PARAMS.kpxy
        kpxy = kpxy+1;
        kpz = 0;
        pxy_RP = sampleInBBox(Cxy,PARAMS.shrink);
        pxy_R0 = footPlane_Rmat'*[pxy_RP 0]'+footPlane_or';
        if PARAMS.verbose then
            mprintf("XY - At iteration %d of %d:\nBase xy_R0 position: [%.4f, %.4f]\n",kpxy,PARAMS.kpxy,pxy_R0(1),pxy_R0(2));
        end
        
        zInterval = cell(1,foot_nb);
        //Compute intersections of the line perpendicular to footPlane, going through pxy_R0, with the WSmi
        line_z = struct('origin',pxy_R0','direction',footPlane_z);
        for i=1:foot_nb
            [boolInterT_i,tMultiple_i,tInterval_i,d_i]=intersectLineWS(WS_R0(:,:,i),shellDesc_AUG(i),line_z,PARAMS.tInc);
            if boolInterT_i then
                tInterval(i).entries = createZInterval(tInterval_i,d_i);
                if PARAMS.verbose & tMultiple_i then
                    mprintf("   T - For leg %d, t lies in %d different intervals", i, size(tInterval(i).entries,1));
                elseif PARAMS.verbose then
                    mprintf("   T - For leg %d, t range is: %.4f to %.4f\n",i,tInterval(i).entries(1),tInterval(i).entries(2));
                end
            else
                if PARAMS.verbose then
                    mprintf("   T - No intersection with leg %d workspace! Resampling pxy_RP...\n",i);
                end
                break;tInterval
            end
        end
        
        if ~boolInterT_i then continue; end
        
        //Sample pz_R0
        [tFinalBool,tFinalInterval] = intersectSetIntervals(tInterval);
        if ~tFinalBool then
            if PARAMS.verbose then
                mprintf("   T - t valid intervals do not intersect! Resammpling pxy_RP...\n");
            end
            continue; 
        end
        
        while kpz<PARAMS.kpz
            kpz = kpz+1;
            kRot = 0;
            t_R0 = sampleFromMultInterval(tFinalInterval);
            if PARAMS.verbose then
                mprintf('T - At iteration %d of %d:\n   Base t_R0 : %.4f\n",kpz,PARAMS.kpz,t_R0);
            end
            
            //Compute intersections of Api arcs and WSmi for the rotation parameter(s)
            base_R0 = pxy_R0'+t_R0*line_z.direction;
            
            P = base_R0;
            
            offset_i = [];
            xOff = [1 0 0]*PARAMS.baseDimensions(1)/2;
            yOff = [0 1 0]*PARAMS.baseDimensions(2)/2;
            
            //Rotation is represented by angle-vector
            //Sample random axis - Watch out because uniform distrib on the three coordinates is not spherically symmetric
            Mean = zeros(3,1);Cov = eye(3,3);
            while kRot<PARAMS.kRot
                kRot = kRot+1;
                rot_axis = grand(1,"mn",Mean,Cov);
                rot_axis = rot_axis/norm(rot_axis);
                rot_axis = rot_axis'; //guarantee to be uniformly distributed on the unit sphere
                if PARAMS.verbose then
                    mprintf('AXIS - At iteration %d of %d:\n   Axis : %.4f %.4f %.4f\n",kRot,PARAMS.kRot,rot_axis(1),rot_axis(2),rot_axis(3));
                end
                
                R_0_EF = footPlane_Rmat; //initial rotation of base
//                R_0_EF = eye(3,3); //no base rotation
                
                angleInter=cell(1,foot_nb);
                arcDesc = struct('origin',base_R0,'normal',rot_axis);
                
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
                    
                    [boolInterAngle_i,angleMultiple_i,angleInter_i] = intersectArcWS(WS_R0(:,:,i),offset_i,R_0_EF,shellDesc(i),arcDesc,PARAMS.aInc);
                    if boolInterAngle_i then
                        angleInter(i).entries = createAngleInterval(angleInter_i);
                        if PARAMS.verbose & angleMultiple_i then
                            mprintf("   ANGLE - For leg %d, angle lies in %d different intervals\n", i, size(angleInter(i).entries,1));
                        elseif PARAMS.verbose then
                            mprintf("   ANGLE - For leg %d, angle range is: %.4f to %.4f\n",i,angleInter(i).entries(1),angleInter(i).entries(2));
                        end
                    else
                        if PARAMS.verbose then
                            mprintf("   ANGLE - No intersection with leg %d workspace! Resampling axis...\n",i);
                        end
                        break;
                    end
                end
                
                if ~boolInterAngle_i then continue; end
                
                //Sample angle
                [angleFinalBool,angleFinalInterval] = intersectSetIntervals(angleInter);
                if ~angleFinalBool then
                    if PARAMS.verbose then
                        mprintf("   ANGLE - angle valid intervals do not intersect! Resampling axis...\n");
                    end
                    continue; 
                end
            
                angle = sampleFromMultInterval(angleFinalInterval);
                if PARAMS.verbose then
                    mprintf('Rotation angle: %.4f",angle*180/%pi);
                end
                
                Q = quatMult(footPlane_Q,createQuaternion(angle,rot_axis));
                RMAT = matrix_fromQuaternion(Q);
                
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
                    
                    IK_target_RLeg = -R_Leg_EF*offset_i' + R_Leg_EF*RMAT'*(STANCE(i).pos'-base_R0'); //the foothold for the ith leg, in the leg base frame
                    IK_target_array(:,i) = IK_target_RLeg;
                    
                    THETA(i,1) = atan(IK_target_RLeg(2),IK_target_RLeg(1));
                    
                    rem = sqrt(IK_target_RLeg(1)**2+IK_target_RLeg(2)**2)-PARAMS.legLength(1);
                    nc3 = IK_target_RLeg(3)**2+rem**2-PARAMS.legLength(2)**2-PARAMS.legLength(3)**2;
                    dc3 = 2*PARAMS.legLength(2)*PARAMS.legLength(3);
                    c3 = nc3/dc3;
                    bool_ik = abs(c3)>1;
                    if bool_ik then
                        if PARAMS.verbose then
                            mprintf("\nIK - NO SOLUTION FOR LEG %s INVERSE KINEMATICS\nResampling axis...\n",STANCE(i).leg);
                        end
                        return;
//                        break;
                    end
                    s3 = factor_elbow*sqrt(1-c3**2); //ELBOw UP
                    THETA(i,3) = factor_t3*atan(s3,c3);
                    
                    THETA(i,2) = factor_t2*(atan(IK_target_RLeg(3),rem)-atan(PARAMS.legLength(3)*s3,PARAMS.legLength(2)+PARAMS.legLength(3)*c3))
                end
                
                if bool_ik then continue; end
                
                SUCCESS=%T;
                if PARAMS.verbose then
                    mprintf("\nSUCCESS!\n");
                end
                return;
            end
    
            if PARAMS.verbose then
                mprintf("AXIS - Reached maximum number of trials, resampling T...\n");
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
