% This file uses GA optimization to tune the parameters of PID controller

% Define system parameters
m = 7.4e-5;
k_m = 0.0281;
k_v = 1.77e-5;
Ke = 1; % Assuming Ke is a constant

% Define the state-space representation
A = [0 1 0; 0 -k_m/m -k_v/m; 0 0 0];
B = [0; Ke/m; 0];
C = [1 0 0];
D = 0;
sys_ss = ss(A, B, C, D);

% Number of variables
n_var = 3;

% Lower and upper bounds for PID gains
lb = [0 0 0];
ub = [100 100 100];

% GA options
ga_opt = gaoptimset('Display','off','Generations',30,'PopulationSize',5,'PlotFcns',@gaplotbestf);

% Objective function
obj_fun = @(x) cost_iae_n(x);

% GA command
[x, best] = ga(obj_fun, n_var, [],[],[],[],lb, ub, [], ga_opt);

%% Run the system again with optimized values
% PID Controller
Kp = x(1);
Ki = x(2);
Kd = x(3);
cont = pid(Kp, Ki, Kd);

% Convert state-space to transfer function
sys_tf = tf(sys_ss);

% Closed-loop system
cl_sys = feedback(cont*sys_tf, 1);

% Time vector
dt = 0.001;
t = 0:dt:5;
sp = ones(1, length(t));
[y, ~] = step(cl_sys, t);

% Plot the results
figure;
plot(t, sp, 'b--','Linewidth',1.5); hold on;
plot(t, y, 'r','Linewidth',1.5);
grid on;
xlabel('Time(s)');
ylabel('Amplitude');
legend('Set-Point', 'GA-PID Result');

% Save optimized results
save('GA_res', 'x', 'best');
