clear; clc;

syms t 
syms m g beta Ixx Iyy Izz real
syms x(t) y(t) z(t) phi(t) gamma(t) theta(t) 


q   = [x(t); y(t); z(t); phi(t); gamma(t); theta(t)]; 
dq  = diff(q, t);
ddq = diff(q, t, 2);

gravity  = [0; 0; -g];
position = [x; y; z];

angularVel = [1,  0,         -sin(gamma);
              0,  cos(phi),   sin(phi)*cos(gamma);
              0, -sin(phi),   cos(phi)*cos(gamma)];
inertiaMatrix = diag([Ixx, Iyy, Izz]);

velocity = dq(1:3);
omega    = dq(4:6);

RotationKineticEnergy    = 0.5 * omega.' * (angularVel.' * inertiaMatrix * angularVel) * omega;
TranslationKineticEnergy = 0.5 * m * (velocity.' * velocity);
PotentialEnergy          = -m * g * q(3);   % = +m*g*z
Dissipative              = 0.5*beta*(dq.'*dq);
KineticEnergy = RotationKineticEnergy+TranslationKineticEnergy;

% Equazioni di Eulero-Lagrange
EL = simplify(diff(gradient(KineticEnergy,dq),t)-gradient(KineticEnergy,q)+gradient(PotentialEnergy,q)+gradient(Dissipative,dq))




%%
NUMBER_MOTOR = 4;
MAX_THRUST = 800;
Kt = 0.8;
Kp = 0.25;
Kr = 0.25;

motorSpeed = zeros(1,NUMBER_MOTOR);



%setenv("ROS_DOMAIN_ID","0");              % uguale a quello dei nodi Python
%node = ros2node("/controllore_matlab");

%pub = ros2publisher(node, "/controllore/motor_speed", "std_msgs/Float64MultiArray");
%comandi = ros2subscriber(node,"/Comandi","std_msgs/Float64MultiArray");
%msg_in = receive(comandi, 5);   % aspetta il primo messaggio (max 5 s)
%thrust = msg_in.data(1);
%pitch  = msg_in.data(2);
%roll   = msg_in.data(3);
%accensione = msg_in.data(6);


