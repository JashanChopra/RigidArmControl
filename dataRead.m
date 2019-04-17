function [time,theta,thetaDot,posRef,voltage,err,Kp,Kd] = dataRead(filename)
% ASEN 2003 - Lab 6 - Control Theory with Rotary Position
% 4/16/2019 - Group 5 - Jashan Chopra, Aiden Wilson, Hugo Stetz, Adam
% Elsayed

%{
This script loads and performs trimming of physical model data

%}

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

% remove extraneous values
fitmodel = fit(time,theta,'smoothingspline');   % fit model with smoothingspline
fx = abs(differentiate(fitmodel, time));        % calculate derivative
cutoff = find(fx < 4);                          % cutoff value
time(cutoff) = []; time = time - time(1);       % remove and reset
theta(cutoff) = []; thetaDot(cutoff) = []; posRef(cutoff) = []; voltage(cutoff) = [];

err = posRef - theta;                 % find residual

end
