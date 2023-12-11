%func for reading sensordata
function sensorData=readSensor(arduinoObj,A0)
    sensorData=readVoltage(arduinoObj,A0);
end