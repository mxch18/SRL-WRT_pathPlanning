function [legPath, success] = localPlanner(config1,config2,s,PARAMS,STANCE,stance_type)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //Outputs a geometric path between config1 and config2 with s steps
    
    //INPUT
    //config1 : row vector. The configuration of a rigid body in 3D in R0
    //              *config(1:3) = position
    //              *config(4:7) = quaternion
    //config2 : idem
    //STANCE: Row array of the current footholds. Contains struct describing
    //        footholds:
    //              *foothold: struct.
    //                  *foothold.leg: string identifying the leg (FR,FL,HR,HL);
    //                  *foothold.pos: row vector. Position of the foot in R0
    //stance_type: 3 or 4 stance
    
    //OUTPUT
    //path : a (5s x sizeOfThetaVector) matrix containing the successive values of the theta for each legs
    
    //TODO :
    //Make s proportional to distance between the configurations
//----------------------------------------------------------------------------//
    legPath = []; success = %F;
    
    rigidBodyPath = interpolate(config1(1:7),config2(1:7),s);
    
    if stance_type == 3 then
        swing_leg_path = interpolatePosition(config1(8:10),config2(8:10),s);
    end
    
    stance_pos_list = STANCE(:).pos;
    stance_pos_array = [];
    for i=1:size(STANCE,2)
        stance_pos_array(i,:) = stance_pos_list(i);
    end
    
    foot_nb = size(STANCE,2)
    
    xOff = [1 0 0]*PARAMS.baseDimensions(1)/2;
    yOff = [0 1 0]*PARAMS.baseDimensions(2)/2;
    THETA = zeros(foot_nb,3);
    
    for j=1:size(rigidBodyPath,1)
        base_R0 = rigidBodyPath(j,1:3);
//        mprintf("\nAt iteration %d, base_R0 : %.2f %.2f %.2f\n",j,base_R0(1),base_R0(2),base_R0(3));
        base_Q = rigidBodyPath(j,4:7);
        for i=1:foot_nb
            if ~isempty(STANCE(i).pos) then
                select STANCE(i).leg
                    case 'FR' then
                        offset_i = xOff + yOff;
                        leg_ef_q = createQuaternion(%pi,[1 1 0]);
                        factor_t2 = -1;
                        factor_t3 = -1;
                        factor_elbow = +1;
                    case 'FL' then
                        offset_i = - xOff + yOff;
                        leg_ef_q = createQuaternion(%pi/2,[0 0 -1]);
                        factor_t2 = +1;
                        factor_t3 = +1;
                        factor_elbow = -1;
                    case 'HR' then
                        offset_i= + xOff - yOff;
                        leg_ef_q = createQuaternion(%pi/2,[0 0 1]);
                        factor_t2 = +1;
                        factor_t3 = +1;
                        factor_elbow = -1;
                    case 'HL' then
                        offset_i = - xOff - yOff;
                        leg_ef_q = createQuaternion(%pi,[1 -1 0]);
                        factor_t2 = -1;
                        factor_t3 = -1;
                        factor_elbow = +1;
                end
                
                transf_Q = quatMult(leg_ef_q,invQuat(base_Q));
                stance_pos = STANCE(i).pos-base_R0;
                
                IK_target_RLeg = -quatMult(quatMult(leg_ef_q,[0 offset_i]),invQuat(leg_ef_q)) + quatMult(quatMult(transf_Q,[0 stance_pos]),invQuat(transf_Q)); //the foothold for the ith leg, in the leg base frame
                IK_target_RLeg = IK_target_RLeg(2:4);
    //            mprintf("\nIK - At iteration %d, for leg %s inverse kinematic, IK_target: %.2f %.2f %.2f\n",j,STANCE(i).leg,IK_target_RLeg(1),IK_target_RLeg(2),IK_target_RLeg(3));
    //            IK_target_array(:,i) = IK_target_RLeg;
                
                THETA(i,1) = atan(IK_target_RLeg(2),IK_target_RLeg(1));
                
                rem = sqrt(IK_target_RLeg(1)**2+IK_target_RLeg(2)**2)-PARAMS.legLength(1);
                nc3 = IK_target_RLeg(3)**2+rem**2-PARAMS.legLength(2)**2-PARAMS.legLength(3)**2;
                dc3 = 2*PARAMS.legLength(2)*PARAMS.legLength(3);
                c3 = nc3/dc3;
    //            mprintf("\nIK - At iteration %d, for leg %s inverse kinematic, c3 = %.4f\n",j,STANCE(i).leg,c3);
                
                if abs(c3)>1 then
                    mprintf("\nIK - At iteration %d, no solution for leg %s inverse kinematic\n",j,STANCE(i).leg);
                    return;
                end
                s3 = factor_elbow*sqrt(1-c3**2); //ELBOw UP
                THETA(i,3) = factor_t3*atan(s3,c3);
                
                THETA(i,2) = factor_t2*(atan(IK_target_RLeg(3),rem)-atan(PARAMS.legLength(3)*s3,PARAMS.legLength(2)+PARAMS.legLength(3)*c3))
            else
                THETA(i,:) = swing_leg_path(j,:);
            end
        end
        
        success = %T;
        
        legPath(j,1:3) = THETA(1,1:3);
        legPath(j,4:6) = THETA(2,1:3);
        legPath(j,7:9) = THETA(3,1:3);
        legPath(j,10:12) = THETA(4,1:3);
        
    end
    
endfunction
