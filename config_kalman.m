%kalman script config

arduinoObj = arduino('COM3'); 

% Kalman filter tuning for the 300 µm precitec sensor
wf1 = 1e-16;
R = (3e-8)^2;

Q1 = calculate_Q(K_x, K_v, m, T_s, wf1);

% Conversion to the extended state-space model integrating Fext into
% the state vector
Ae = [mod_etat_lin1.A mod_etat_lin1.B; 0 0 0];
Be = [mod_etat_lin1.B ; 0];
Ce = [1 0 0];
mod_lin_etendu1 = ss(Ae, Be, Ce, 0);
mod_etendu_1 = c2d(mod_lin_etendu1, T_s);
A1 = mod_etendu_1.a;
B1 = mod_etendu_1.b;
C1 = mod_etendu_1.c;

% Sending values to the Arduino
writePWMVoltage(arduinoObj, 'D9', A1);  % Assuming D9 is connected to DS_A1
writePWMVoltage(arduinoObj, 'D10', B1); % Assuming D10 is connected to DS_B1
writePWMVoltage(arduinoObj, 'D11', Q1); % Assuming D11 is connected to DS_Q1
writePWMVoltage(arduinoObj, 'D12', R);  % Assuming D12 is connected to DS_R1

% Cleanup
clear arduinoObj;
