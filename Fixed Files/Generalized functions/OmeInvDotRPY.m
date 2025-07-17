function OmeDot = OmeInvDotRPY(phi,theta,phip,thetap)
OmeDot = zeros(3,3);   
OmeDot(1,1) = -sin(phi)*cos(theta)*phip - cos(phi)*sin(theta)*thetap;
OmeDot(1,2) = -cos(phi)*phip;
OmeDot(2,1) = cos(phi)*cos(theta)*phip - sin(phi)*sin(theta)*thetap;
OmeDot(2,2) = -sin(phi)*phip;
OmeDot(3,1) = -cos(theta)*thetap;
OmeDot(3,2) = 0;

end

