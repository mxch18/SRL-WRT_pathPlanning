function [x,y,z]=halfSph(orig,r,alp,tet,direction)
    
    direction = direction/norm(direction);
    
    alpOffset = atan(direction*[0;1;0],direction*[1;0;0]);
    tetOffset = atan(direction*[0;0;1],direction*[0;1;0]);
    Rprel = [0 0 1;0 1 0;-1 0 0];
    Ralp = [cos(alpOffset) -sin(alpOffset) 0;sin(alpOffset) cos(alpOffset) 0;0 0 1];
    Rtet = [cos(tetOffset) 0 -sin(tetOffset);0 1 0;sin(tetOffset) 0 cos(tetOffset)];
    
    lalp = length(alp);
    ltet = length(tet);
    x = zeros(1,lalp*ltet);
    y = zeros(1,lalp*ltet);
    z = zeros(1,lalp*ltet);
    p = zeros(3,1);
    
    for j=1:ltet
        for i=1:lalp
            p(1) = r*cos(alp(i))*cos(tet(j));
            p(2) = r*sin(alp(i))*cos(tet(j));
            p(3) = r*sin(tet(j));
            
            p = Rtet*Ralp*Rprel*p;
            
            x(j+ltet*(i-1))= p(1)+orig(1);
            y(j+ltet*(i-1))= p(2)+orig(2);
            z(j+ltet*(i-1))= p(3)+orig(3);
        end
    end
    
    
endfunction
