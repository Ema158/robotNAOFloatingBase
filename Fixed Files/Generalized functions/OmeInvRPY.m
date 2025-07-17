function OmeInv = OmeInvRPY(phi,theta)
OmeInv = [ 0    -sin(phi)   cos(phi)*cos(theta);%Khalil book
           0     cos(phi)   sin(phi)*cos(theta);
           1    0   -sin(theta)];
end