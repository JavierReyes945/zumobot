%% Author: Javier Reyes
%  Date: 21.06.2017

clear all, clc, close all

%  Physical parameters of the system //TODO: Re-define values
g = 9.8;            % Gravity constant [ m/s^2 ]
M = 1;              % Mass of cart [ kg ]
m = 0.2;            % Mass of pendulum [ kg ]
b = 0;              % Friction constant [ N/m/s ]
L = 12.5e-2;        % Lenght of pendulum to center of gravity [ m ]
I = m*(L^2);        % Moment of intertia (pendulum) [ kg.m^2 ]
R = 2.3e-2;         % Radius of pulley [ m ]
tcm = 0.5;          % Time constant of the motor [ s ]
Km = 17;            % Gain motor [ rad/s/V ]
Kf = 9/pi;          % Gain of feedback [ V/rad/s ]

%  Linearized approximation transfer function of Zumo
Kp = 1/((M+m)*g);
Ap = 1/((m*g*L*(M+m))/((M+m)*(L+m*(L^2))-(m*L)^2));
numZumo = [Kp];
denZumo = [Ap -1];
iptf = tf(numZumo, denZumo, 'InputName', 'Force', ...
                            'OutputName', 'angular position');

%  Overall transfer function of actuation mechanism
numMotor = [Km*(M+m)*R 0];
denMotor = [tcm 1];
mtf = tf(numMotor, denMotor, 'InputName', 'Error voltage', ...
                            'OutputName', 'angular position');

%  Transfer function of the whole system
numSys = [Kf*Kp*Km*R*(M+m) 0];
denSys = [Ap*tcm Ap -tcm -1];
zmtf = tf(numSys, denSys, 'InputName', 'Error voltage', ...
                          'OutputName', 'angular position');

%  Zero-pole map of the open loop system
h1 = rlocusplot(zmtf);
ph1 = getoptions(h1);
ph1.Title.String = 'Zero-pole map of the open loop system';
ph1.Title.FontSize = 12;
setoptions(h1, ph1)

%  Model creation
sys = 'model3';
open_system(sys);

%  Setting the input parameters
set_param(''model3/DesInput', ...
          'Time', '2', ...      %  Step time
          'Before', '0', ...   %  Initial value
          'After', '1');        %  Final value

%  Setting the motor transfer function
set_param('model3/Motor1', ...
          'Numerator', mat2str(numMotor, 5), ...
          'Denominator', mat2str(denMotor, 5));

%  Setting the zumobot transfer function
set_param('model3/Zumo1', ...
          'Numerator', mat2str(numZumo, 5), ...
          'Denominator', mat2str(denZumo, 5));

%  Setting the motor transfer function
set_param('model3/Motor2', ...
          'Numerator', mat2str(numMotor, 5), ...
          'Denominator', mat2str(denMotor, 5));

%  Setting the zumobot transfer function
set_param('model3/Zumo2', ...
          'Numerator', mat2str(numZumo, 5), ...
          'Denominator', mat2str(denZumo, 5));

%  Setting the motor transfer function
set_param('model3/Motor3', ...
          'Numerator', mat2str(numMotor, 5), ...
          'Denominator', mat2str(denMotor, 5));

%  Setting the zumobot transfer function
set_param('model3/Zumo3', ...
          'Numerator', mat2str(numZumo, 5), ...
          'Denominator', mat2str(denZumo, 5));

%  Run the Model
sym(sys);
