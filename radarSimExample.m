%% FMCW Example -----------------------------------------------------------
% Based on Automotive Radar Example from Matlab
%   Copyright 2012-2015 The MathWorks, Inc.
clear
close all
tic

% Vehicle placement ------------------------------------------------------
% Our system
Nsweep = 8;
% changing the bandwith does not change the power level/ SIR calculations
tm = 10e-3;      
radar_speed = 0;    %m/s
radar_init_pos = [0;0;0.5]; %m
car_speed = 30/((tm)*(Nsweep-1)); % m/s, 
car_init_pos = [10;0;0.5];   %m
itfer_speed = 0;
itfer_init_pos = [10, 3.048, 0.5]'% Car lanes are about 10 ft --> 3.6576 m
target = 'car'
wave = 1

% dur = (tm)*(Nsweep-1);
% System waveform parameters ---------------------------------------------
fc = 2.445e9;  
c = 3e8;   
rangeMax = 80;   
bw = 70e6; %range2bw(rangeRes,c); % Find range
LPmixer = 28e3;
load('SampleRadiationPatterns.mat', 'TPLink'); % Antenna Model
rad_pat = TPLink; clear TPLink;

% Interferer waveform parameters -----------------------------------------
if wave == 1
    bw_INT = bw;%*0.2577; %40e6;
    tm_INT = tm;%/2;
elseif wave == 2
    bw_INT = bw*0.2577; %40e6;
    tm_INT = tm;%/2;
elseif wave == 3
    bw_INT = bw*0.2577; %40e6;
    tm_INT = tm/2;
end

% Antenna Model Set Up ---------------------------------------------------
% % MIT Values
txPower = 0.65; %db2pow(5)*1e-3;                     % in watts
txLossFactor = 0;                             % in dB **TODO**

% IF Power = -28 dBm
rxNF = 4.5;                                    % in dB
rxLossFactor = 0;                             % in dB **TODO
toc

% Turn on and off sections of code ---------------------------------------
PLOT.VEHICLES = 0;
PLOT.POWER = 0;
PLOT.ACCURACY = 0;
PLOT.BEATSIGNAL = 0;
PLOT.CHIRP = 0;
MUTUAL_INTERFERENCE= 1;
ONE_WAY_CHANNEL = 0;
SAVE = 0;
PHASE_SHIFT = 0;
fileName = 'filename.mat';

%% Run the function
[~, beatsignal, fs_bs] = radarSim(fc, tm, tm_INT, rangeMax, bw, bw_INT, Nsweep, LPmixer,...
    rad_pat, radar_speed, radar_init_pos, car_speed, car_init_pos,...
    itfer_speed, itfer_init_pos, txPower, txLossFactor,rxNF,...
    rxLossFactor,...
    PLOT, MUTUAL_INTERFERENCE, ...
    PHASE_SHIFT, SAVE, fileName, target);

%%
%  plotBeatSignal(beatsignal, fs_bs,1, 1)
% load('scen23_wave1_10m.mat', 'beatsignal', 'fs_bs')
[output] = calcSimSIR(beatsignal, fs_bs)
