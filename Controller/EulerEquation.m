clear; clc;

syms t 
syms m g beta Ixx Iyy Izz real
syms x(t) y(t) z(t) theta_x(t) theta_y(t) theta_z(t) 


q   = [x(t); y(t); z(t); theta_x(t); theta_y(t); theta_z(t)]; 
dq  = diff(q, t);
ddq = diff(q, t, 2);

gravity  = [0; 0; -g];
position = [x; y; z];

W_body = [1,  0,            -sin(theta_y);
          0,  cos(theta_x),  sin(theta_x)*cos(theta_y);
          0, -sin(theta_x),  cos(theta_x)*cos(theta_y)];
inertiaMatrix = diag([Ixx, Iyy, Izz]);

velocity = dq(1:3);
dtheta    = dq(4:6);
omega_body = W_body * dtheta;

RotationKineticEnergy    = 0.5 *(omega_body.' * inertiaMatrix * omega_body);
TranslationKineticEnergy = 0.5 * m * (velocity.' * velocity);
PotentialEnergy          = m * g * q(3);   
Dissipative              = 0.5*beta*(dq.'*dq);
KineticEnergy = RotationKineticEnergy+TranslationKineticEnergy;

% Equazioni di Eulero-Lagrange
EL = simplify(diff(gradient(KineticEnergy,dq),t)-gradient(KineticEnergy,q)+gradient(PotentialEnergy,q)+gradient(Dissipative,dq));

%forze
syms k b l w0 w1 w2 w3 real

Tau_tot          =   k    *   (w0^2 + w1^2 + w2^2 + w3^2);
Tau_theta_x      =   k*l  *   ((w0^2+w2^2)-(w3^2+w1^2));
Tau_theta_y      =   k*l  *   ((w2^2+w1^2)-(w3^2+w0^2));
Tau_theta_z      =   b    *   ((w2^2+w3^2)-(w0^2+w1^2));

Force = m*position'*position + g;

R = [cos(theta_y)*cos(theta_z)      sin(theta_x)*sin(theta_y)*cos(theta_z)-cos(theta_x)*sin(theta_z)        cos(theta_x)*sin(theta_y)*cos(theta_z)+sin(theta_z)*sin(theta_x)
     cos(theta_y)*sin(theta_z)      sin(theta_x)*sin(theta_y)*sin(theta_z)+cos(theta_x)*cos(theta_z)        cos(theta_x)*sin(theta_y)*sin(theta_z)-sin(theta_x)*cos(theta_z)
     -sin(theta_y)                  sin(theta_x)*cos(theta_y)                                               cos(theta_x)*cos(theta_y)];

Q = [R * [0; 0; Tau_tot];
     W_body.' * [Tau_theta_x; Tau_theta_y; Tau_theta_z]];

equations = EL == Q;    % 6 equazioni

%%
% 1. Sostituisci le funzioni del tempo con variabili simboliche algebriche
syms x_s y_s z_s tx_s ty_s tz_s real
syms dx_s dy_s dz_s dtx_s dty_s dtz_s real
syms ddx_s ddy_s ddz_s ddtx_s ddty_s ddtz_s real

q_s   = [x_s; y_s; z_s; tx_s; ty_s; tz_s];
dq_s  = [dx_s; dy_s; dz_s; dtx_s; dty_s; dtz_s];
ddq_s = [ddx_s; ddy_s; ddz_s; ddtx_s; ddty_s; ddtz_s];
clear; clc;

syms t 
syms m g beta Ixx Iyy Izz real
syms x(t) y(t) z(t) theta_x(t) theta_y(t) theta_z(t) 


q   = [x(t); y(t); z(t); theta_x(t); theta_y(t); theta_z(t)]; 
dq  = diff(q, t);
ddq = diff(q, t, 2);

gravity  = [0; 0; -g];
position = [x; y; z];

W_body = [1,  0,            -sin(theta_y);
          0,  cos(theta_x),  sin(theta_x)*cos(theta_y);
          0, -sin(theta_x),  cos(theta_x)*cos(theta_y)];
inertiaMatrix = diag([Ixx, Iyy, Izz]);

velocity = dq(1:3);
dtheta    = dq(4:6);
omega_body = W_body * dtheta;

RotationKineticEnergy    = 0.5 *(omega_body.' * inertiaMatrix * omega_body);
TranslationKineticEnergy = 0.5 * m * (velocity.' * velocity);
PotentialEnergy          = m * g * q(3);   
Dissipative              = 0.5*beta*(dq.'*dq);
KineticEnergy = RotationKineticEnergy+TranslationKineticEnergy;

% Equazioni di Eulero-Lagrange
EL = simplify(diff(gradient(KineticEnergy,dq),t)-gradient(KineticEnergy,q)+gradient(PotentialEnergy,q)+gradient(Dissipative,dq));

%forze
syms k b l w0 w1 w2 w3 real

Tau_tot          =   k    *   (w0^2 + w1^2 + w2^2 + w3^2);
Tau_theta_x      =   k*l  *   ((w0^2+w2^2)-(w3^2+w1^2));
Tau_theta_y      =   k*l  *   ((w2^2+w1^2)-(w3^2+w0^2));
Tau_theta_z      =   b    *   ((w2^2+w3^2)-(w0^2+w1^2));

Force = m*position'*position + g;

R = [cos(theta_y)*cos(theta_z)      sin(theta_x)*sin(theta_y)*cos(theta_z)-cos(theta_x)*sin(theta_z)        cos(theta_x)*sin(theta_y)*cos(theta_z)+sin(theta_z)*sin(theta_x)
     cos(theta_y)*sin(theta_z)      sin(theta_x)*sin(theta_y)*sin(theta_z)+cos(theta_x)*cos(theta_z)        cos(theta_x)*sin(theta_y)*sin(theta_z)-sin(theta_x)*cos(theta_z)
     -sin(theta_y)                  sin(theta_x)*cos(theta_y)                                               cos(theta_x)*cos(theta_y)];

Q = [R * [0; 0; Tau_tot];
     W_body.' * [Tau_theta_x; Tau_theta_y; Tau_theta_z]];

equations = EL == Q;    % 6 equazioni
% Sostituzione (prima le derivate seconde, poi le prime, poi le variabili)
eq_alg = subs(equations, [ddq; dq; q], [ddq_s; dq_s; q_s]);

% 2. Isola la matrice di massa M(q) e il vettore dei termini noti F(q, dq, u)
% Le equazioni hanno forma: M(q)*ddq = F(q, dq, u)
[M_sym, F_sym] = equationsToMatrix(eq_alg, ddq_s);

% Risolvi rispetto a ddq (per un 6x6 simbolico è quasi istantaneo)
ddq_explicit = simplify(M_sym \ F_sym);

% 3. Genera automaticamente il file .m per Simulink
u_s      = [w0; w1; w2; w3];
params_s = [m; g; beta; Ixx; Iyy; Izz; k; b; l];

matlabFunction(ddq_explicit, ...
    'File', 'quadrotor_dynamics', ...
    'Vars', {q_s, dq_s, u_s, params_s});




%%



