function Ome = OmeRPY(phi,theta)
Ome = [ cos(phi)/cos(theta)    sin(phi)/cos(theta)   0;
        -sin(phi)              cos(phi)            0;
       cos(phi)*tan(theta)    sin(phi)*tan(theta)   1];
end

