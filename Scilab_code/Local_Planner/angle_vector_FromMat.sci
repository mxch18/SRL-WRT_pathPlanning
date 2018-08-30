function [angle,vector] = angle_vector_FromMat(rMat)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //Outputs the quaternion qOut = q1*q2;
    
    //INPUT
    //rMat: a rotation matrix. Assumed orthogonal
    
    //OUTPUT
    //qOut: quaternion. Row vector
    
    //TODO : add orthogonality test ?
    
//----------------------------------------------------------------------------//
    vector = zeros(3,1);angle=0;
    
    tmpMat = rMat-rMat'; //R-R' = 2*sin(angle)*[u] ->skew-symmetric matrix
    
    [f,err] = assert_checkalmostequal(tmpMat,zeros(3,3),0,10e-6);
    
    if f then //tmpMat is zero matrix
        tmpMat_2 = rMat+eye(3,3);
        if rank(tmpMat_2)==1 then
            angle = %pi;
            if tmpMat_2(1,1)==0 then
                if tmpMat_2(2,2)==0 then
                    if tmpMat_2(3,3)==0 then
                        mprintf('Error: vector is null'); //mathematically impossible case but you never know
                        return;
                    end
                    vector = tmpMat_2(:,3);
                    vector = vector';
                    vector = vector/norm(vector);
                    return;
                end
                vector = tmpMat_2(:,2);
                vector = vector';
                vector = vector/norm(vector);
                return;
            end
            vector = tmpMat_2(:,1);
        else
            angle = 0;
            vector = rand(3,1);
        end
    else
        vector(1)=tmpMat(3,2);
        vector(2)=tmpMat(1,3);
        vector(3)=tmpMat(2,1);
        angle = acos((trace(rMat)-1)/2); //see paper
    end
    
    vector = vector';
    vector = vector/norm(vector);
    
endfunction
