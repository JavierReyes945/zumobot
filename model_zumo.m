%% Author: Javier Reyes
%  Date: 21.06.2017

clear all, clc, close all

%  Physical parameters of the system
%  TODO: Re-define values
g = 9.8;				% Gravity constant [ m/s² ]
m = 0.25;				% Mass of cart+pendulum [ kg ]]
b = 0;					% Friction constant [ N/m/s ]
L = 8.6e-2;		% Lenght of pendulum to center of gravity [ m ]
I = m*(L^2);		% Moment of intertia (pendulum) [ kg.m² ]
R = 1.3e-2;			% Radius of pulley [ m ]
tcm = 0.5;			% Time constant of the motor [ s ]
Km = 17;				% Gain motor [ rad/s/V ]
Kf = 1;					% Gain of feedback [ V/rad/s ]

%  Selection of modeling method
%  (1) Forces approach
%  (2) Euler-Lagrange approach
sel = 1;

%% Summing forces approach
if sel==1
	%  Model creation
	sys = 'model';
	open_system(sys);

	%  Linearized approximation transfer function of Zumo
	%
	%  theta(s)          Kp                   1                  (      (M+m)mgl      )
	%  -------- = ----------------- ; Kp = ------- ; Ap = +- sqrt( ------------------ )
	%    F(s)     (1/(Ap²))s² - 1          (M+m)g                ( (M+m)(l+ml²)-(ml)² )
	%
	%  Due to the dual mass in the model, an approximation is made: (M+m) = m
	%
	%  theta(s)          Kp                 1                 (       m²gl       )
	%  -------- = ----------------- ; Kp = ---- ; Ap = +- sqrt( ---------------- )
	%    F(s)     (1/(Ap²))s² - 1           mg                ( (m)(l+ml²)-(ml)² )

	Kp = 1/((m)*g);
	Ap = ((m*(L+m*(L^2))-(m*L)^2)/(m^2*g*L)); % Power and inverse already calculated
	numZumoA = [Kp];
	denZumoA = [Ap -1];
	iptfA = tf(numZumoA, denZumoA, 'InputName', 'force', ...
																'OutputName', 'angular position');

	%  Overall transfer function of motor and actuation mechanism
	%
	%   F(s)            mrs
	%  ------ = Km * ---------
	%   V(s)          (ts+1)

	numMotorA = [Km*m*R 0];
	denMotorA = [tcm 1];
	mtfA = tf(numMotorA, denMotorA, 'InputName', 'voltage', ...
																	'OutputName', 'force');

	%  Transfer function of the whole system
	stfA = series(mtfA, iptfA);
	ftfA = feedback(stfA, 1);

	%  Zero-pole map of the open loop system
	figure
	h1 = rlocusplot(stfA);
	ph1 = getoptions(h1);
	ph1.Title.String = 'Zero-pole map of the open loop system';
	ph1.Title.FontSize = 12;
	setoptions(h1, ph1)

	%  Setting the motor and zumobot transfer function
	for i = 1:3
	  set_param(sprintf('model/Motor%d', i) , ...
	            'Numerator', mat2str(numMotorA, 5), ...
	            'Denominator', mat2str(denMotorA, 5));

	  set_param(sprintf('model/Zumo%d', i) , ...
	            'Numerator', mat2str(numZumoA, 5), ...
	            'Denominator', mat2str(denZumoA, 5));
    end
end
%% Euler-Lagrange approach
if sel==2
	%  Linearized approximation transfer function of Zumo
	%
	%  theta(s)     -s^2
	%  -------- = --------
	%    X(s)     Ls^2 - g

	numZumoB = [-1 0 0];
	denZumoB = [L 0 -g];
	iptfB = tf(numZumoB, denZumoB, 'InputName', 'displacement', ...
	                              'OutputName', 'angular position');

	%  Overall transfer function of motor and actuation mechanism
	%
	%   X(s)      Km
	%  ------ = --------
	%   V(s)    s(ts+1))

	numMotorB = [Km];
	denMotorB = [tcm 1 0];
	mtfB = tf(numMotorB, denMotorB, 'InputName', 'voltage', ...
	                                'OutputName', 'displacement');

	%  Transfer function of the whole system
	stfB = series(mtfB, iptfB);
	ftfB = feedback(stfB, 1);

	%  Zero-pole map of the open loop system
	figure
	h2 = rlocusplot(stfB);
	ph2 = getoptions(h2);
	ph2.Title.String = 'Zero-pole map of the open loop system';
	ph2.Title.FontSize = 12;
	setoptions(h2, ph2)

	%  Setting the motor and zumobot transfer function
	for i = 1:3
	  set_param(sprintf('model/Motor%d', i) , ...
	            'Numerator', mat2str(numMotorB, 5), ...
	            'Denominator', mat2str(denMotorB, 5));

	  set_param(sprintf('model/Zumo%d', i) , ...
	            'Numerator', mat2str(numZumoB, 5), ...
	            'Denominator', mat2str(denZumoB, 5));
    end
end
