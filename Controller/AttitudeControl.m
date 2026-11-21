clear all; close all;clc

m = 1.5; g = 9.8; beta = 0.05;
Ixx = 0.035; Iyy = 0.07; Izz = 0.098;
k = 8.55e-6; b = 1.37e-7; l = 0.25;

par = [m; g; beta; Ixx; Iyy; Izz; k; b; l];

% Riferimento di test: hovering a quota z = 1m, x = 0, y = 0, yaw = 0
ref_test = [0; 0; 1; 0];