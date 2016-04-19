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
radar_speed_x = 0;                    %m/s
radar_init_pos = [0;0;0.5];           %m
tgt_speed_x = 30/((tm)*(Nsweep-1));   %m/s, 
tgt_init_pos = [10;0;0.5];            %m
% itfer_speed_x = 0;
% itfer_init_pos = [10, 3.048, 0.5]';% Car lanes are about 10 ft --> 3.6576 m
itferData = [10, 3.048, 3;
             15, -3.048,0];
target = 'car';
wave = 1;

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
PLOT.VEHICLES = 1;
PLOT.POWER = 0;
PLOT.ACCURACY = 1;
PLOT.PREVIEW = 1;
PLOT.BEATSIGNAL = 0;
PLOT.CHIRP = 0;
ONE_WAY_CHANNEL = 0;
SAVE = 0;
PHASE_SHIFT = 0;
MUTUAL_INTERFERENCE = 1;
TARGET = 1;
SCEN_TYPE = '1'; %1,2,3,4,'custom'
fileName = 'filename.mat';


%% Preview vehicle positions
% get the duration of the simulation
[radarPos, tgtPos, itferPos,...
    radarVel, tgtVel, itferVel] = prevEnv(Nsweep, tm,...
    radar_init_pos, tgt_init_pos, itferData,...
    radar_speed_x, tgt_speed_x, PLOT.PREVIEW, MUTUAL_INTERFERENCE, TARGET);

%% Run the function
[~, beatsignal, fs_bs] = radarSim(fc, tm, tm_INT, rangeMax, bw, bw_INT, Nsweep, LPmixer,...
    rad_pat, radarPos, itferPos, tgtPos, radarVel, itferVel, tgtVel,...
    txPower, txLossFactor,rxNF,...
    rxLossFactor,...
    PLOT, MUTUAL_INTERFERENCE, TARGET, ...
    PHASE_SHIFT, SAVE, fileName, target);

% [~, beatsignal, fs_bs] = radarSim(fc, tm, tm_INT, rangeMax, bw, bw_INT, Nsweep, LPmixer,...
%     rad_pat, radar_speed_x, radar_init_pos, tgt_speed_x, tgt_init_pos,...
%     itfer_speed_x, itfer_init_pos, txPower, txLossFactor,rxNF,...
%     rxLossFactor,...
%     PLOT, MUTUAL_INTERFERENCE, ...
%     PHASE_SHIFT, SAVE, fileName, target);

%%
plotBeatSignal(beatsignal, fs_bs,PLOT.BEATSIGNAL, MUTUAL_INTERFERENCE)
% load('scen23_wave1_10m.mat', 'beatsignal', 'fs_bs')
[output] = calcSimSIR(beatsignal, fs_bs)
