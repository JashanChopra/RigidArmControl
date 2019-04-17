function [time,theta,thetaDot,posRef,voltage,error,Kp,Kd] = dataRead(filename)
% ASEN 2003 - Lab 6 - Control Theory with Rotary Position
% 4/16/2019 - Group 5 - Jashan Chopra, Aiden Wilson, Hugo Stetz, Adam
% Elsayed

%{
This script loads and performs trimming of physical model data

%}
clc; clear all; clf;

% Read Data & Get Initial Conditions
data = load(filename);   % load data file
Kd = data(1,10); Kp = data(1,8);        % gather control constants

time = data(:,1);                       % gather time vector
time = time - time(1);                  % zero it out
time = time / 1000;                     % convert to [s]

theta = data(:,2);                      % angular position [rad]
thetaDot = data(:,4);                   % angular velocity [rad/s]
posRef = data(:,6);                     % desired position [rad]
voltage = data(:,7);                    % Ouput voltage [V]

error = posRef - theta;                 % find residual

end
