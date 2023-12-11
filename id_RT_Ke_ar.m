
arduinoObj = arduino('COM3');  

% Check if in open loop and valid_u is zero
closed_loop = readDigitalPin(arduinoObj, 'D2');  
valid_u = readDigitalPin(arduinoObj, 'D3'); 

if (closed_loop == 1 && valid_u == 0)
    % Set K_e = 1
    writePWMVoltage(arduinoObj, 'D5', 5);  % Assuming K_e control pin is connected to PWM pin D5

    % Read coil current
    i_bob = readVoltage(arduinoObj, 'A0');  % Assuming the coil current is connected to analog pin A0

    % Acquisition of estimated force for a single instant
    numSamples = 5000;
    hatF = mean(zeros(1, numSamples));  % Placeholder, modify as needed
    
    % Calculate Ke
    K_e = i_bob / hatF;

    % Update in Arduino (replace with actual code based on your setup)
    writePWMVoltage(arduinoObj, 'D5', K_e);  % Assuming K_e control pin is connected to PWM pin D5

    % Reset I_bob (replace with actual code based on your setup)
    writeVoltage(arduinoObj, 'A1', 0);  % Assuming I_bob reset pin is connected to analog pin A1
else
    disp(' ');
    disp('ERROR!');
    disp('Switch to open loop and set "valid_u" to zero, then retry');
    disp(' ');
end

% Cleanup
clear arduinoObj;
