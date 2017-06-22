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

%  Linearized approximation transfer function IP
Kp = 1/((M+m)*g);
Ap = 1/((m*g*L*(M+m))/((M+m)*(L+m*(L^2))-(m*L)^2));
numZumo = [Kp];
denZumo = [Ap -1];
iptf = tf(numZumo, denZumo, 'InputName', 'Force', ...
                            'OutputName', 'angular position');

%  Overall transfer function of actiation mechanism
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
h1 = rlocusplot(zmtf)
ph1 = getoptions(h1);
ph1.Title.String = 'Zero-pole map of the open loop system';
ph1.Title.FontSize = 12;
setoptions(h1, ph1)

%  Model creation
sys = 'zumoModel';
new_system(sys);
open_system(sys);

%  Setup values for block positions and sizes
x = 30;
y = 30;
w = 120;
h = 60;
offset = 180;
hCounter = 1;
vCounter = 1;

%  Creating a block for the step function
pos = [x y+h/4 x+w y+h*.75];
add_block('built-in/Step',[sys '/Setpoint'],'Position',pos, ...
          'Time', '3', ...      %  Step time
          'Before', '0', ...   %  Initial value
          'After', '1');        %  Final value

%  Creating a block for the sum
pos = [x+(offset*hCounter) y*vCounter x+(offset*hCounter)+w (y*vCounter)+h];
add_block('built-in/Sum',[sys '/sum1'],'Position',pos, 'IconShape', 'round', ...
          'Inputs', '|+-');
hCounter = hCounter + 1;

%  Adding the connection between sum and motor
add_line(sys,'Setpoint/1','sum1/1','autorouting','on');

%  Creatin a block for the motor
pos = [x+(offset*hCounter) y*vCounter x+(offset*hCounter)+w (y*vCounter)+h];
add_block('built-in/TransferFcn',[sys '/Motor'],'Position',pos, ...
          'Numerator', mat2str(numMotor), 'Denominator', mat2str(denMotor));
hCounter = hCounter + 1;

%  Adding the connection between sum and motor
add_line(sys,'sum1/1','Motor/1','autorouting','on');

%  Creatin a block for the Zumobot
pos = [x+(offset*hCounter) y*vCounter x+(offset*hCounter)+w (y*vCounter)+h];
add_block('built-in/TransferFcn',[sys '/Zumo'],'Position',pos, ...
          'Numerator', mat2str(numZumo), 'Denominator', mat2str(denZumo));
hCounter = hCounter + 1;

%  Adding the connection between motor and Zumobot
add_line(sys,'Motor/1','Zumo/1','autorouting','on');

%  Creatin a block for the Scope
pos = [x+(offset*hCounter) y*vCounter x+(offset*hCounter)+w (y*vCounter)+h];
add_block('built-in/Scope',[sys '/Scope1'],'Position',pos, ...
          'Open', 'on');
hCounter = hCounter + 1;

%  Adding the connection between Zumobot and Scope
add_line(sys,'Zumo/1','Scope1/1','autorouting','on');

%  Adding the connection between Zumobot and add block
add_line(sys,'Zumo/1','sum1/2','autorouting','on');

%  Run the Model
sym(sys);
