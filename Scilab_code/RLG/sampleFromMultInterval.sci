function smpl = sampleFromMultInterval(multipleIntv)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //
    
    //INPUT
    //multipleIntv: matrix of intervals. Size is Nx2.
    
    //OUTPUT
    //sample: a uniform random sample in one of the interval of the input
    
//----------------------------------------------------------------------------//
    
    N = size(multipleIntv,1);
    lowBnd = zeros(1,N);
    highBnd = zeros(1,N);
    sizeIntv = zeros(1,N);
    sizeIntvs = 0;
    
    lowBnd(1) = multipleIntv(1,1);
    highBnd(1) = multipleIntv(1,2);
    sizeIntv(1) = abs(highBnd(1)-lowBnd(1));
    sizeIntvs = sizeIntvs + sizeIntv(1);
    
    for i = 2:N
        lowBnd(i) = multipleIntv(i,1);
        highBnd(i) = multipleIntv(i,2);
        size_i = abs(highBnd(i)-lowBnd(i));
        sizeIntv(i) = sizeIntv(i-1) + size_i;
        sizeIntvs = sizeIntvs + size_i;
    end
    
    weightedBern = rand()*sizeIntvs;
    
    for i =1:N
        if weightedBern <= sizeIntv(i) then
            smpl = lowBnd(i) + rand()*(highBnd(i)-lowBnd(i));
            break;
        end
    end
    
    
endfunction
