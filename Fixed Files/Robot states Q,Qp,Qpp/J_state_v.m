function J_h = J_state_v(robot)  
%Computation of the jacobian of h function for virtual constraint
J_h = zeros(robot.joints-6,robot.joints);
%% Foot position
J_h(1:3,:)=robot.J_RAnkle;
J_h(7:9,:)=robot.J_LAnkle;
%% Foot yaw,pitch,roll
Foot = robot.T(:,:,8)*robot.foot_Rf';
phi=atan2(Foot(2,1),Foot(1,1));
theta=atan2(-Foot(3,1),cos(phi)*Foot(1,1)+sin(phi)*Foot(2,1));
J_wR = zeros(3,robot.joints);
Roll = robot.q(4);
Pitch = robot.q(5);
% J_wR(1:3,4:6) = OmeInvRPY(Roll,Pitch);
J_wR(1:3,4:6) = inv(OmeRPY(Roll,Pitch));
for i = 2:7
    J_wR(:,i-1+6)=robot.T(1:3,3,i); % crea la matriz J_foot,w = [0a2, 0a3, 0a4, 0a5, ..., 0a13, 0 ... 0] (3x22)
end
J_h(4:6,:)=OmeRPY(phi,theta)*J_wR;

Foot = robot.T(:,:,15)*robot.foot_Lf';
phi=atan2(Foot(2,1),Foot(1,1));
theta=atan2(-Foot(3,1),cos(phi)*Foot(1,1)+sin(phi)*Foot(2,1));
J_wL = zeros(3,robot.joints);
Roll = robot.q(4);
Pitch = robot.q(5);
% J_wL(1:3,4:6) = OmeInvRPY(Roll,Pitch);
J_wL(1:3,4:6) = inv(OmeRPY(Roll,Pitch));
for i = 9:14
    J_wL(:,i-2+6)=robot.T(1:3,3,i); % -2 por el marco no actuado en el pie derecho
end
J_h(10:12,:)=OmeRPY(phi,theta)*J_wL;
J_h(13:22,13+6:22+6) = eye(10,10);
J_h(23:24,23+6:24+6) = eye(2,2);
end

