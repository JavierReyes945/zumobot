%% Author: Javier Reyes
%  Date: 21.06.2017

clear all, clc, close all

%  Model creation
sys = 'zumoModel';
new_system(sys);
open_system(sys);

%  Values from transfer functions
g = 9.8;
l = 0.27;               % TODO: Lenght of Zumobot
trq = 0.155;
Ia = 120e-3;
numMotor = [trq/Ia];      % TODO: Check constant - Kma != Km
denMotor = [55927.5, 1];   % TODO: Check constant - Tm != Max torque
numZumo = [(-1/g), 1];
denZumo = [l, 0, -g];

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
          'Time', '0', ...      %  Step time
          'Before', '10', ...   %  Initial value
          'After', '0');        %  Final value

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
