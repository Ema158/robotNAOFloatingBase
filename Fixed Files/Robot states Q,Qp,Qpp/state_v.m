
function h = state_v(robot) %%Corregir
%Computation of the h function for virtual constraint
baseDof = 6;
h = zeros(24,1);
%% Right Leg joints
h(1:6) = robot.q(1+baseDof:6+baseDof);
%% Left Leg joints
h(7:12) = robot.q(7+baseDof:12+baseDof);
%%
FootR = robot.T(:,:,15)*robot.foot_Lf'; % The orientation (in pitch, roll and yaw angles) of the free foot is obtained from the transformation matrix  0T14
% Frame 14 is taken as it, since in zero potition (i.e. q=0 ) this frame has the same orientation as frame 0
h(12)=atan2(FootR(2,1),FootR(1,1));  % Yaw (psi) rotación  alrededor del eje Z
h(11)=atan2(-FootR(3,1),cos(h(12))*FootR(1,1)+sin(h(12))*FootR(2,1)); % Pitch (theta) rotación  alrededor del eje Y
h(10)=atan2(sin(h(12))*FootR(1,3)-cos(h(12))*FootR(2,3),-sin(h(12))*FootR(1,2)+cos(h(12))*FootR(2,2)); % Roll (phi) rotación  alrededor del eje X
%
%%
h(13:22) = robot.q(13+baseDof:22+baseDof); %arms
h(23:24) = robot.q(23+baseDof:24+baseDof); %head
%
end

