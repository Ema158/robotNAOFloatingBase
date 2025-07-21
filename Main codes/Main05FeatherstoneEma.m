close all;
clear all
clc
tic; % Start the timer
% ---------------------------------------------------------------------------------------------
currentfolder = pwd; % save current path
cd ..                % go one folder down (go out of the current folder and stay in the previous one)
add_paths;           % add all folders where all files are founded in order to acces to them
cd(currentfolder);   % return to the original path
% ---------------------------------------------------------------------------------------------

echo on
% Testing Essential dynamics standing on one foot
% ==============================
% Creation: 28/Ago/2024
% Last modification: -/--/--
% --------------------------------------------------------------------------------------------------
% Robot stands in one foot and regulates 
% -----------------------------------------------------------------------------------------------------
% --------------------------------------------------------------------------------------------------
% 
%
echo off

global gait_parameters
global robot coms tau
tau = zeros(24,1);
coms=0;
anim=true;
% Parameters
% ---------------------------------------------------------
Nao_param = ParamTest1();
% ---------------------------------------------------------
gait_parameters = Nao_param.gait_parameters;
% 
% T = gait_parameters.T;
% g = gait_parameters.g;
% z0 = gait_parameters.z_i;
% S = gait_parameters.S;
% D = gait_parameters.y_ffoot_i;

% CHOSING CONTROLLED VARIABLE FILES
% -------------------------------------------------------------------------------------------------
global OptionContVar  % Option to chose the controlled variables "hd", "hpd" and "hppd"
% 1 -> Controlled variables defined by polynomials w.r.t. time, IMPACT is considered. Files: "hd_Polyn", "hpd_Polyn_t" and "hppd_Polyn_t". 
% 2 -> Controlled variables defined by cycloidal motion w.r.t. time, IMPACT is NOT considered. Files: "hd_CycMotion_t", "hpd_CycMotion_t" and "hppd_CycMotion_t". 
% 3 -> Controlled variables defined by polynomials w.r.t. "x" of the CoM, IMPACT is considered. Files: "hd_Polyn", "hpd_Polyn_x" and "hppd_Polyn_x". 
OptionContVar = Nao_param.ControlledVariableOption;
% -------------------------------------------------------------------------------------------------

% GENERAL OPTIONS
% ----------------------------------------------------
DataName = 'InfoNAO_FloatingBase5';
NameAnim = ['anim_', DataName];
robot = genebot();
time_step = 0.01;
T = 15;
samples = T/time_step;
current_time = 0;
X0 = zeros(60,1);
X0(1:30) = robot.q; %[0pB,0etaB,qJ]
X0(31:60) = zeros(30,1); %[0vB,0wB,qDJ]
% X0(31) = 2;
% X0(33) = 3;
% X0(34) = 3;
Xt = zeros(samples+1,length(X0));
tauT = zeros(samples+1,24);
CoM = zeros(samples+1,3);
t = zeros(samples+1,1);
t(1) = current_time;
Xt(1,:) = X0';
tauT(1,:) = zeros(1,24);
CoM(1,:) = robot.CoM';
for i=1:samples
    timespan = [current_time, current_time + time_step];
    Xtaux = ode4(@FloatingBaseSimulationV5FeatherstoneEma,timespan,X0);
    Xt(i+1,:) = Xtaux(end,:);
    tauT(i+1,:) = tau';
    CoM(i+1,:) = robot.CoM';
    t(i+1) = current_time + time_step;
    X0 = Xtaux(end,:)';
    current_time = current_time + time_step;
end
robot_draw(robot)
figure(2)
plot(t,Xt(:,1))
hold on
plot(t,Xt(:,2))
% plot(t,Xt(:,3))
plot(t,CoM(:,1),'k')
plot(t,CoM(:,2),'k')
% plot(t,CoM(:,3),'k')
grid on
%
figure(3)
plot(t,Xt(:,4))
hold on
plot(t,Xt(:,5))
plot(t,Xt(:,6))
grid on

figure(4)
plot(t,tauT(:,1))
hold on
plot(t,tauT(:,2))
grid on
%
qt = Xt(:,1:30)';
%
samplesPerSecond = 5;
samplesAnimation = round(T*samplesPerSecond);
%% Walking ANIMATION
% ==============================================================================
figure(5)
if anim    
    dataS = cell(1,1); % Sampled joint positions
    disp('Animation...')
    % This part is just to make the animation faster, the larger "n" the slower and finer the animation
    n = samplesAnimation; % Number of samples of vector
        qS = sampling(qt,n); % sampling of joint position "q"       
        % We don't need the joint velocities to draw the robot, so we don't sample the velocity "qp"
        dataS{1,1} = qS;
    framerate = 5;
    animationStandingOnefoot(dataS,1,NameAnim,framerate); % ("parametro"= Numero de pasos a observar al final de la simulaci?n y a grabar en el video
    disp(['Animation stored as: ' NameAnim]);
    disp('----------------------------------');
end
% ==============================================================================
%% End of the code
% ----------------------------------------------------------------------------------------------
cd ..                  % Go down one folder (go out from the current folder and stay in the previous one)
remove_paths;          % Remove the paths of the folders added at the begining
cd(currentfolder);     % Return to the original folder
% ----------------------------------------------------------------------------------------------
