function q = InvGeometricHZDtime(qf,robot,gait_parameters,t)
%Numerical solver of the inverse geometric model assuming the virtual
%constraint are respected.
baseDof = 6;
q_init=robot.q;
Case = 'position';
% "Case", "gait_parameters" and "t" are used inside "OptionDesiredTrajectory"
OptionDesiredTrajectory;

Q = [state_vTest1(robot);robot.CoM(1:3);robot.q(4:6)];  % Conjunto de posiciones OPERACIONALES reales Q = [h(q)^T, q_f^T]^T
Qdes = [hd; qf];

%Numerical threshold
Thre = 0.2;
global nQ
% ---------------------------
dQ = Qdes-Q;
nQ = max(abs(dQ));
i=0;
Analyze = false;  % 1-> Plot the robot each iteration, so we can analize who the algorithm is converging 0-> We don't analyze anything
if Analyze
    % Just to check how the algorithm adjust the position step by step until reach the desired position
    % -----------------------------------------------------
    % Posición incial
    robot = robot_move(robot,q_init);
    figure
    robot_draw(robot);
    axis equal
    % pause;
    figure
    % ------------------------
end

while nQ>1e-10
    if nQ>Thre
        dQ = dQ./norm(Q).*Thre;
    end
%     dq = InvKinematic(dQ,robot);
    dq = InvKinematicTest1(dQ,robot);
    %Next iteration
    q=robot.q+dq;
    robot = robot_move(robot,q);
%     Q = [state_v(robot);robot.CoM(1:3);robot.q(4:6)]; 
    Q = [state_vTest1(robot);robot.CoM(1:3);robot.q(4:6)];  % 
    dQ = Qdes-Q;
    nQ = max(abs(dQ));
    i=i+1;
    if Analyze
        % Just to check how the algorithm adjust the position step by step until reach the desired position 
        % -----------------------------------------------------
        i
        hold off
        plot(0,0)
        hold off
        robot_draw(robot);
        axis equal
        view(3)
%         fprintf('Ankle Roll  ->  q1 = %f, q12 = %f \n',rad2deg(q(1+6)),rad2deg(q(12+6)))
%         fprintf('Ankle Pitch ->  q2 = %f, q11 = %f \n',rad2deg(q(2+6)),rad2deg(q(11+6)))
%         fprintf('knee Pitch  ->  q3 = %f, q10 = %f \n',rad2deg(q(3+6)),rad2deg(q(10+6)))
%         fprintf('Hip Pitch   ->  q4 = %f,  q9 = %f \n',rad2deg(q(4+6)),rad2deg(q(9+6)))
%         fprintf('Hip Roll    ->  q5 = %f,  q8 = %f \n',rad2deg(q(5+6)),rad2deg(q(8+6)))
%         fprintf('Hip Yaw     ->  q6 = %f,  q7 = %f \n',rad2deg(q(6+6)),rad2deg(q(7+6)))
        [[1:30]' Qdes(1:30) Q(1:30)]
        pause;
        % ------------------------
    end
    if i == 200
        warning('Out of the workspace!');
        global OutOfWorkSpace
        if isempty(OutOfWorkSpace)
            OutOfWorkSpace = 1;
        end
        figure(99)
        title('Out of workspace');
        [[1:30]' Qdes Q];
        nQ; 
        robot_draw(robot);
        robot = robot_move(robot,q_init);
        disp('Robot configuration re-initialized')
%         figure(100)
%          robot_draw(robot,0,0);
% %         pause;
        break
    end
end
q=robot.q;
end

