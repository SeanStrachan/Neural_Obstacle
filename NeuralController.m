function [voltage_left, voltage_right] = NeuralController(Left_sensor ,Right_sensor,net)

sensors = [Left_sensor;Right_sensor];
N_N = net(sensors);
voltage = [4,4];
%disp(N_N);
%display(brian(1));
if N_N == [1;0]
    voltage = [6,-1];
    
elseif N_N == [0;1]
    voltage = [-1,6];

elseif N_N == [1;1]
    voltage = [4,4];
    
elseif N_N == [0;0]
    voltage = [1,1];
    
end
    
voltage_left=voltage(1);
voltage_right=voltage(2);
%disp(voltage)