%real time data acquisition and parameters identification with arduino
arduinoObj=arduino('COM3');  %input port arduino

%PWM for current coil control
coilPin='D9';
configurePin(arduinoObj,coilPin, 'PWM');

%init conditions
writePWMDutyCycle(arduinoObj,coilPin,0); %zero coil current, pause before data acquisition
pause(1);

%data acqui
num_samples=5000;
sampling_inter=0.001;
time_vector=(0:num_samples-1)*sampling_inter;

data=zeros(num_samples,1);
for i=1:num_samples
    data(i)=readSensor(arduinoObj,A0);
    pause(sampling_inter);
end

%plot results
figure;
plot(time_vector,data);
xlabel('Time in secs');
ylable('sensor reading');
title("real time data acquisition")

%system identification

inputData=zeros(num_samples,1); %parsing input data
identData=iddata(data,inputData, sampling_inter);
identified_model=pem(identData,2,'ss','can','Tolerance',0.001);

%calulating Kv, and Kx from identified model

Kx=-identified_model.A(2,1);
Kv=-identified_model.A(2,2);


%linear model

s = tf('s');
mod_lin = 1/K_x/(1+ K_v/K_x*s + m/K_x*s^2);
Al = [0 1; -K_x/m -K_v/m];
Bl = [0; 1/m];
Cl = [1 0];
mod_etat_lin1 = ss(Al,Bl,Cl,0);

%write data to arduino
K_x_pin='A1' %replace with pin number in arduino

K_x_value = Kx;  
writeVoltage(arduinoObj, K_x_pin, K_x_value);


clear arduinoObj;







