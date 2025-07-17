
function h = state_vTest1(robot)
%Computation of the h function for virtual constraint
baseDof = 6;
h = zeros(24,1);
%% Right Leg joints
h(1:6) = robot.q(1+baseDof:6+baseDof);
%% Left Leg joints
h(7:12) = robot.q(7+baseDof:12+baseDof);
%%
h(13:22) = robot.q(13+baseDof:22+baseDof); %arms
h(23:24) = robot.q(23+baseDof:24+baseDof); %head
%
end

