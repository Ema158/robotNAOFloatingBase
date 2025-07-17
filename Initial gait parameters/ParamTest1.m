function   Nao_param = ParamTest1()
%% Gait parameters 
% ============================
T = 1.5;   %8  1.5    % Time step
g = 9.81;        % acceleration of gravity
% --------------
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parametres for evolution of the free foot
gait_parameters.T = T;               % Period of the step
gait_parameters.g = g;               % gravity acceleration
%% RLeg
gait_parameters.q1_0 = 0;
gait_parameters.q1_f = 0;

gait_parameters.q2_0 = 0;
gait_parameters.q2_f = 0;

gait_parameters.q3_0 = -0.3;
gait_parameters.q3_f = -0.3;

gait_parameters.q4_0 = 0.3;
gait_parameters.q4_f = 0.3;

gait_parameters.q5_0 = 0;
gait_parameters.q5_f = 0;

gait_parameters.q6_0 = 0;
gait_parameters.q6_f = 0;
%% LLeg
gait_parameters.q7_0 = 0;
gait_parameters.q7_f = 0;

gait_parameters.q8_0 = 0;
gait_parameters.q8_f = 0;

gait_parameters.q9_0 = -0.3;
gait_parameters.q9_f = -0.3;

gait_parameters.q10_0 = 0.3;
gait_parameters.q10_f = 0.3;

gait_parameters.q11_0 = 0;
gait_parameters.q11_f = 0;

gait_parameters.q12_0 = 0;
gait_parameters.q12_f = 0;

%% RArm
gait_parameters.q13_0 = 1.6;
gait_parameters.q13_f = 1.6;

gait_parameters.q14_0 = 0;
gait_parameters.q14_f = 0;

gait_parameters.q15_0 = 1.6;
gait_parameters.q15_f = 1.6;

gait_parameters.q16_0 = 0.2;
gait_parameters.q16_f = 0.2;

gait_parameters.q17_0 = 0;
gait_parameters.q17_f = 0;
%% LArm
gait_parameters.q18_0 = -1.6;
gait_parameters.q18_f = -1.6;

gait_parameters.q19_0 = 0;
gait_parameters.q19_f = 0;

gait_parameters.q20_0 = 1.6;
gait_parameters.q20_f = 1.6;

gait_parameters.q21_0 = -0.2;
gait_parameters.q21_f = -0.2;

gait_parameters.q22_0 = 0;
gait_parameters.q22_f = 0;
%% Head
gait_parameters.q23_0 = 0;
gait_parameters.q23_f = 0;

gait_parameters.q24_0 = 0;
gait_parameters.q24_f = 0;
%% CHOSING CONTROLLED VARIABLE FILES
% -------------------------------------------------------------------------------------------------
% Option to chose the controlled variables "hd", "hpd" and "hppd"
% 1 -> Controlled variables defined by polynomials w.r.t. time, IMPACT can be considered. Files: "hd_Polyn", "hpd_Polyn_t" and "hppd_Polyn_t". 
% 2 -> Controlled variables defined by cycloidal motion w.r.t. time, IMPACT is NOT considered. Files: "hd_CycMotion_t", "hpd_CycMotion_t" and "hppd_CycMotion_t". 
% 3 -> Controlled variables defined by polynomials w.r.t. "x" of the CoM, IMPACT can be considered. Files: "hd_Polyn", "hpd_Polyn_x" and "hppd_Polyn_x". 
OptionContVar = 1;
% -------------------------------------------------------------------------------------------------                    
% Creating a structure for the parameters
Nao_param.gait_parameters = gait_parameters;
Nao_param.ControlledVariableOption = OptionContVar;
