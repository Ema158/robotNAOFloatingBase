function [qfpp, JPhi, JPhipPhip, NE1, NE2, NE3, qfp] = Desired_qfpp_FloatingBaseTest1(robot_i,X,gait_param_i,t)
global robot gait_parameters
robot = robot_i;
gait_parameters = gait_param_i;
% =======================================================================
%   Cálculo de JPhi y JPhipPhi
% -----------------------------------------------------------------------
qf = X(1:6);
qfp = X(7:12);
[q,qD,JPhi,JQ,~] = JointsPosVel_from_CoMPosVel_HZDtime(qf,qfp,robot,gait_parameters,t);
robot = robot_move(robot,q);

robot.qD = qD; % Updating of the robot velocities since they will be used for the computing of several subsequent matrices
JQpqp = JQp_qp(robot);

Case = 'acceleration';
% "Case", "gait_parameters", "qfp" and "t" are used inside "OptionDesiredTrajectory"
OptionDesiredTrajectory;
% ------------------------------
JdpPhip = [dt_dhd_Phi; 
          zeros(6,6+phiDim)]*Phip;
            
JPhipPhip = JQ\(JdpPhip - JQpqp);

% ====================================================================================
% Cálculo de pares y fuerza y momento de reacción para PARTE de la acceleración qpp.
% ------------------------------------------------------------------------------------
% Se toma en cuenta que
% qpp = Jf*qpp + gf = Jf(col1)*xpp + Jf(col2)*ypp  + gf.. entonces, se calculan las pares y fuerzas para
%  "Jf(col1) + gf", "Jf(col2)+ gf"  y "gf" de tal forma que podamos despejar y encontrar "qfpp"
% -----------------------------------------------------------------------
% Utilizando la OPCION III utilizada en "RomeoNewtonEuler02.m"  (Se usa el algoritmo de NE 3 VECES, -opción 2 en el reporte-)
% -----------------------------------------------------
term1 = JPhi(:,1) + JPhipPhip; %
term2 = JPhi(:,2) + JPhipPhip; %
term3 = JPhi(:,3) + JPhipPhip;
term4 = JPhi(:,4) + JPhipPhip;
term5 = JPhi(:,5) + JPhipPhip;
term6 = JPhi(:,6) + JPhipPhip;
term7 = JPhipPhip; %

[F1,M1,Tau1]= Newton_Euler(q,qD,term1);  % [F1 M1 Tau1]' = De*(Jf1 + gf) + He
[F2,M2,Tau2]= Newton_Euler(q,qD,term2);  % [F2 M2 Tau2]' = De*(Jf2 + gf) + He
[F3,M3,Tau3]= Newton_Euler(q,qD,term3);  % [F2 M2 Tau2]' = De*(Jf3 + gf) + He
[F4,M4,Tau4]= Newton_Euler(q,qD,term4);  % [F2 M2 Tau2]' = De*(Jf3 + gf) + He
[F5,M5,Tau5]= Newton_Euler(q,qD,term5);  % [F2 M2 Tau2]' = De*(Jf3 + gf) + He
[F6,M6,Tau6]= Newton_Euler(q,qD,term6);  % [F2 M2 Tau2]' = De*(Jf3 + gf) + He
[F7,M7,Tau7]= Newton_Euler(q,qD,term7);  % [F2 M2 Tau2]' = De*(Jf3 + gf) + He

% =================================================================================================
% Si quisiéramos encontrar la fuerza F momento M y pares Tau reales debido a q, qp y qpp, deberíamos utilizar qppf de tal
% forma que 
%  F = (F1-F3)*xpp + (F2-F3)*ypp + F3 ;  
%  M = (M1-M3)*xpp + (M2-M3)*ypp + M3 ;  
%  Tau = (Tau1-Tau3)*xpp +  (Tau2-Tau3)*ypp + Tau3;
% Sin embargo lo que queremos encontrar es la trayectoria de "qfpp" que hará que el ZMPd = ZMP.
% AHORA, usando F0a despejaremos "xpp" y "ypp" y luego lo compararemos con XDD
% ---------------------------------------------------------------------------
%  vect1 = [F1 - F3; M1 - M3; Tau1 - Tau3]; % Vector de 37 x 1
%  vect2 = [F2 - F3; M2 - M3; Tau2 - Tau3]; % Vector de 37 x 1
%  vect3 = [F3; M3; Tau3]; % Vector de 37 x 1
%  vect0 = [F; M; Tau]; % Estos son los pares y fuerzas completos obtenidos con NE(q,qp,qpp)
%  Jfe = [vect1 vect2]; % Matriz de 37 x 2
%  XDDn = inv(Jfe'*Jfe)*Jfe'*(vect0 - vect3); 

% =======================================================================
%   Cálculo de qfpp
% -----------------------------------------------------------------------
NE1 = [F1;M1;Tau1];
NE2 = [F2;M2;Tau2];
NE3 = [F3;M3;Tau3];
NE4 = [F4;M4;Tau4];
NE5 = [F5;M5;Tau5];
NE6 = [F6;M6;Tau6];
NE7 = [F7;M7;Tau7];
DeBar = [NE1-NE7, NE2-NE7, NE3-NE7, NE4-NE7, NE5-NE7,NE6-NE7];
HeBar = NE7;

%
Q = gait_parameters.Q;
A = DrBar;
b = -HrBar;
options = optimset('Display', 'off');
k = evaluateTref(robot.tRef,t);
varThetaRef = varThetaPdControlZmpTerm2(robot,xRef,[robot.Zref(1,k);robot.Zref(2,k)],ZMPd);
p = -varThetaRef'*Q;
%
Q2 = zeros(length(Q)+4,length(Q)+4);
Q2(1:length(Q),1:length(Q)) = Q;
Q2(length(Q)+1:end,length(Q)+1:end) = zeros(4,4);
p2 = [p,zeros(1,4)];
A2 = [A, zeros(2,4)];

mu = gait_parameters.mu;
AFC = zeros(3,length(Q)+4);
AFC(1:3,1:12) = DeBar(1:3,:);
muMatrix = [-mu,0,mu,0;
            0,-mu,0,mu;
            -1,-1,-1,-1];
AFC(:,13:16) = muMatrix;
bFC = [-HeBar(1);-HeBar(2);-HeBar(3)];
AeqC = [A2;AFC];
beqC = [b;bFC];

Aineq = zeros(length(Q)+4,length(Q)+4);
Aineq(13:end,13:end) = -eye(4,4);
bineq = zeros(16,1);
tic
desVar = quadprog(Q2,p2,Aineq,bineq,AeqC,beqC,[],[],[],options); 
elapsedTime = toc; % Stop the timer and get the elapsed time
fprintf('The script took %.2f seconds to run.\n', elapsedTime);
% if abs(t-0.2)<1e-6&&gait_parameters.boolDisturbance==1 %Perturbacion en x
%     qfpp(1) = qfpp(1) + gait_parameters.magnitudeDisturbance;
% end
qfpp = desVar(1:12);
if abs(t-0.05)<1e-6&&gait_parameters.boolDisturbance==1 %Perturbacion en y
    qfpp(2) = qfpp(2) - gait_parameters.magnitudeDisturbance;
end
CAMp = Acam*qfpp + bcam;
qfpp(13:14) = CAMp;

