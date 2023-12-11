function J = cost_iae_n(x)
   
    
    % Define system parameters
    m = 7.4e-5;
    k_m = 0.0281;
    k_v = 1.77e-5;
    Ke = 0.0067; % Assuming Ke is a constant
    
    % Define the state-space representation
    A = [0 1 0; 0 -k_m/m -k_v/m; 0 0 0];
    B = [0; Ke/m; 0];
    C = [1 0 0];
    D = 0;
    sys_ss = ss(A, B, C, D);
    
    % Convert state-space to transfer function
    sys_tf = tf(sys_ss);
    
    % PID Controller
    Kp = x(1);
    Ki = x(2);
    Kd = x(3);
    cont = pid(Kp, Ki, Kd);
    
    % Closed-loop system
    cl_sys = feedback(cont*sys_tf, 1);
    
    % Time vector
    dt = 0.001;
    t = 0:dt:5;
    
    % Step response
    [y, ~] = step(cl_sys, t);
    
    % Calculate the error
    e = 1 - y;
    
    % Calculate the integral of absolute error (IAE)
    J = sum(abs(e)*dt);
end
