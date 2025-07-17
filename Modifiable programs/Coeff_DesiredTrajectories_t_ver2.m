
function PolyCoeff = Coeff_DesiredTrajectories_t_ver2(robot,gait_parameters)

% In this code the coefficients of the polynomials that define the trajectory for step of the robot are computed

T = gait_parameters.T;

q1_0 = gait_parameters.q1_0;
q2_0 = gait_parameters.q2_0;
q3_0 = gait_parameters.q3_0;
q4_0 = gait_parameters.q4_0;
q5_0 = gait_parameters.q5_0;
q6_0 = gait_parameters.q6_0;
q1_f = gait_parameters.q1_f;
q2_f = gait_parameters.q2_f;
q3_f = gait_parameters.q3_f;
q4_f = gait_parameters.q4_f;
q5_f = gait_parameters.q5_f;
q6_f = gait_parameters.q6_f;

q7_0 = gait_parameters.q7_0;
q8_0 = gait_parameters.q8_0;
q9_0 = gait_parameters.q9_0;
q10_0 = gait_parameters.q10_0;
q11_0 = gait_parameters.q11_0;
q12_0 = gait_parameters.q12_0;
q7_f = gait_parameters.q7_f;
q8_f = gait_parameters.q8_f;
q9_f = gait_parameters.q9_f;
q10_f = gait_parameters.q10_f;
q11_f = gait_parameters.q11_f;
q12_f = gait_parameters.q12_f;

q13_0 = gait_parameters.q13_0;
q14_0 = gait_parameters.q14_0;
q15_0 = gait_parameters.q15_0;
q16_0 = gait_parameters.q16_0;
q17_0 = gait_parameters.q17_0;
q13_f = gait_parameters.q13_f;
q14_f = gait_parameters.q14_f;
q15_f = gait_parameters.q15_f;
q16_f = gait_parameters.q16_f;
q17_f = gait_parameters.q17_f;

q18_0 = gait_parameters.q18_0;
q19_0 = gait_parameters.q19_0;
q20_0 = gait_parameters.q20_0;
q21_0 = gait_parameters.q21_0;
q22_0 = gait_parameters.q22_0;
q18_f = gait_parameters.q18_f;
q19_f = gait_parameters.q19_f;
q20_f = gait_parameters.q20_f;
q21_f = gait_parameters.q21_f;
q22_f = gait_parameters.q22_f;

q23_0 = gait_parameters.q23_0;
q23_f = gait_parameters.q23_f;
q24_0 = gait_parameters.q24_0;
q24_f = gait_parameters.q24_f;

% INITIAL DESIRED VALUES in position and velocity of the CONTROLLED VARIABLES
% ======================================================================================
% Desired positions at the beginnning of the step (time t=0)
h0_d = zeros(24,1);
h0_d(1) = q1_0;
h0_d(2) = q2_0;
h0_d(3) = q3_0;
h0_d(4) = q4_0;
h0_d(5) = q5_0;
h0_d(6) = q6_0;

h0_d(7) = q7_0;
h0_d(8) = q8_0;
h0_d(9) = q9_0; 
h0_d(10) = q10_0; 
h0_d(11) = q11_0;
h0_d(12) = q12_0;

h0_d(13) = q13_0;
h0_d(14) = q14_0;
h0_d(15) = q15_0; 
h0_d(16) = q16_0; 
h0_d(17) = q17_0;

h0_d(18) = q18_0;
h0_d(19) = q19_0;
h0_d(20) = q20_0; 
h0_d(21) = q21_0; 
h0_d(22) = q22_0;

h0_d(23) = q23_0;
h0_d(24) = q24_0;
% Desired velocities at the beginnning of the step (time t=0)
hp0_d = zeros(24,1);
% CHECK if the transition is taken into account or not to consider the values after impact or the initial desired values
transition = gait_parameters.transition;
if transition
    % The initial positions and velocities are based on the current state of the robot after the transition (impact or not)
    [Qplus, QpPlus] = current_states(robot);
    IniPos = Qplus(1:12); 
    IniVel = QpPlus(1:12);
else % The polynomials are computed for ideal initial conditions 
    % (this is useful for compute the final state of the robot for compute the first step)        
    IniPos = h0_d;
    IniVel = hp0_d;
end

% FINAL DESIRED VALUES in position and velocity of the CONTROLLED VARIABLES
% ======================================================================================
% The Desired values for position of the controlled variables "hd" at the beginnning of the step (time t=T) are the 
% same that the ideal ones at the beginning of the step for almost all the variables, except for:
% -----------------------------------------------------------------------------
hT_d = h0_d;
hT_d(1) = q1_f;
hT_d(2) = q2_f;
hT_d(3) = q3_f;
hT_d(4) = q4_f;
hT_d(5) = q5_f;
hT_d(6) = q6_f;

hT_d(7) = q7_f;
hT_d(8) = q8_f;
hT_d(9) = q9_f; 
hT_d(10) = q10_f; 
hT_d(11) = q11_f;
hT_d(12) = q12_f;

hT_d(13) = q13_f;
hT_d(14) = q14_f;
hT_d(15) = q15_f; 
hT_d(16) = q16_f; 
hT_d(17) = q17_f;

hT_d(18) = q18_f;
hT_d(19) = q19_f;
hT_d(20) = q20_f; 
hT_d(21) = q21_f; 
hT_d(22) = q22_f;

hT_d(23) = q23_f;
hT_d(24) = q24_f;
% Desired values for velocity of the controlled variables "hd" at the end of the step (time t=T)
% -----------------------------------------------------------------------------
hpT_d = zeros(24,1);

for i=1:24
    PosD = [0,IniPos(i);
            T, hT_d(i)];
    VelD = [0, IniVel(i);
            T, hpT_d(i)];
    %prueba ema
    AccD = [0, 0;
            T, 0];
   PolyCoeff.(['hd', int2str(i)]) = findPolyCoeff(PosD,VelD,AccD); % posd,veld,accd
end

end
