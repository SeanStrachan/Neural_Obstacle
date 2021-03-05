%RUNMODEL - Main code to run robot model for ENG5009 class
%
%   NOTE: Students should only change the code between *---* markers
%
% Syntax: RunModel
%
% Inputs: none
% Outputs: none
% 
% Other m-files required: 
%   Sensor.m
%   WallGeneration.m
%   DynamicalModel.m
%   DrawRobot.m
%
% Author: Dr. Kevin Worrall
% Last revision: 06-01-2021

%% Preamble
%close all;
%clear all;
%clc;

%% Simulation setup
% Time
simulationTime_total = 50;           % in seconds *------* YOU CAN CHANGE THIS
stepSize_time = 0.05;               % in seconds 

% Initial states and controls
voltage_left  = 4;                  % in volts *------* YOU CAN CHANGE THIS
voltage_right = 4;                  % in volts *------* YOU CAN CHANGE THIS

state_initial = zeros(1,24);        % VARIABLES OF IMPORTANCE:
                                    % state_initial(13): forward velocity,    v, m/s
                                    % state_initial(18): rotational velocity, r, rad/s
                                    % state_initial(19): current x-position,  x, m
                                    % state_initial(20): current y-position,  y, m
                                    % state_initial(24): heading angle,       psi, rad
                                    
% Environment
canvasSize_horizontal = 10;
canvasSize_vertical   = 10;
stepSize_canvas       = 0.01;

% *------------------------------------------*
% Setup of neural network
%% define inputs and target outputs
[net] = Perceptron();
% *------------------------------------------*

%% Create Environment
obstacleMatrix = zeros(canvasSize_horizontal / stepSize_canvas, canvasSize_vertical / stepSize_canvas);

% Generate walls
% --> the variable "obstacleMatrix" is updated for each added wall
[wall_1, obstacleMatrix] = WallGeneration( -1,  1, 1.2, 1.2, 'h', obstacleMatrix); 
[wall_2, obstacleMatrix] = WallGeneration( -3, -3,  -2,   2, 'v', obstacleMatrix);
[wall_3, obstacleMatrix] = WallGeneration( 3, 3,  -2,   2, 'v', obstacleMatrix);
[wall_4, obstacleMatrix] = WallGeneration( -3, -2,  -2,   0, 'h', obstacleMatrix);
% *---------------------------*
%  YOU CAN ADD MORE WALLS HERE
% *---------------------------*


%% Main simulation
% Initialize simulation 
timeSteps_total = simulationTime_total/stepSize_time;
state = state_initial;
time = 0;
%Perceptron();
% Run simulation
for timeStep = 1:timeSteps_total
    % Assign voltage applied to wheels
    voltages = [voltage_left; voltage_left; voltage_right; voltage_right];
    
    % *-------------------------------------*
    %  YOU CAN ADD/CHANGE YOUR CONTROLS HERE
    sensorOut=Sensor(state(timeStep, 19), state(timeStep, 20), state(timeStep,24), obstacleMatrix);
    disp(sensorOut)
    Left_sensor = sensorOut(1);
    Right_sensor = sensorOut(2);
    [voltage_left,voltage_right] = NeuralController(Left_sensor, Right_sensor,net);
    voltS=[voltage_left,voltage_right];
    disp(voltS);
    %voltage_right = voltages(2);

    % *-------------------------------------*
    
    % Run model *** DO NOT CHANGE
    [state_derivative(timeStep,:), state(timeStep,:)] = DynamicalModel(voltages, state(timeStep,:), stepSize_time);   
    
    % Euler intergration *** DO NOT CHANGE
    state(timeStep + 1,:) = state(timeStep,:) + (state_derivative(timeStep,:) * stepSize_time); 
    time(timeStep + 1)    = timeStep * stepSize_time;
    
    % Plot robot on canvas  *------* YOU CAN ADD STUFF HERE
    figure(1); clf; hold on; grid on; axis([-5,5,-5,5]);
    DrawRobot(0.2, state(timeStep,20), state(timeStep,19), state(timeStep,24), 'b');
    plot(wall_1(:,1), wall_1(:,2),'k-');
    plot(wall_2(:,1), wall_2(:,2),'k-'); 
    plot(wall_3(:,1), wall_3(:,2),'k-');
    plot(wall_4(:,1), wall_4(:,2),'k-'); 
    xlabel('y, m'); ylabel('x, m');
end

%% Plot results
% *----------------------------------*
%  YOU CAN ADD OR CHANGE FIGURES HERE
%  don't forget to add axis labels!
% *----------------------------------*

%figure(2); hold on; grid on;
%plot(state(:,20), state(:,19));

%figure(3); hold on; grid on;
%plot(time, state(:,19));

%figure(4); hold on; grid on;
%plot(time, state(:,24));
