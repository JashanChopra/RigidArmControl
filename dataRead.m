function [time,theta,thetaDot,posRef,voltage,error,Kp,Kd] = dataRead()
% ASEN 2003 - Lab 6 - Control Theory with Rotary Position
% 4/16/2019 - Group 5 - Jashan Chopra, Aiden Wilson, Hugo Stetz, Adam
% Elsayed

%{
This script loads and performs trimming of physical model data

%}
clc; clear all; clf;

% Read Data & Get Initial Conditions
data = load('Group5Tests_Kp30_Kd05');   % load data file
Kd = data(1,10); Kp = data(1,8);        % gather control constants

time = data(:,1);                       % gather time vector
time = time - time(1);                  % zero it out
time = time / 1000;                     % convert to [s]

theta = data(:,2);                      % angular position [rad]
thetaDot = data(:,4);                   % angular velocity [rad/s]
posRef = data(:,6);                     % desired position [rad]
voltage = data(:,7);                    % Ouput voltage [V]

error = posRef - theta;                 % find residual

%% Plotting to understand Data
yyaxis left
plot(time,theta)
hold on
plot(time,posRef)
titleText = sprintf('Theta over time - Kp: %3.1f - Kd: %1.2f',Kp,Kd');
title(titleText)
ylabel('Theta [Rad]')
xlabel('Time [s]')

yyaxis right
plot(time,thetaDot)
ylabel('Rotational Velocity [rad/s]')
legend('Actual Theta','Reference Position','Rotational Velocity')

end
