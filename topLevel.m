close all
df = 300;           % Bandwidth (Hz)
fc = 500;          % Center frequency (Hz)
fs = (fc+df/2)*20;  % Sampling rate (Hz)
Tm = 3;             % Modulation Index (s)
theta = pi/2;       % Angle of Arrival for object (radians)
distance = 3*10^3;  % Distance of object (m)
velocity = 10^6;    % Velocity of object (m/s)
gamma = df/Tm;  % Chirp Rate

[tx_signal, reflected_signal, fdoppler, tshift] = objectProcessingFunct( df, fc, fs, Tm, theta, distance, velocity );

[ dist, vel, f_D, f_B ] = rxBlock( tx_signal, reflected_signal, gamma, df );