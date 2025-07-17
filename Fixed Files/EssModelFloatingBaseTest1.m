function [t,Xt] = EssModelFloatingBaseTest1(X0,gait_parameters)
T = gait_parameters.T;
time_step = 0.01;
current_time = 0;
samples = T/time_step; 
Xt = zeros(samples+1,length(X0));
Xt(1,:) = X0';
for i=1:samples
    timespan = [current_time, current_time + time_step];
    Xtaux = ode4(@dynam_FloatingBaseTest1,timespan,X0);
    
    Xt(i+1,:) = Xtaux(end,:);
    X0 = Xtaux(end,:)';
    current_time = current_time + time_step;
end
t = 0:time_step:T;
disp('-------------------------------------------');                  