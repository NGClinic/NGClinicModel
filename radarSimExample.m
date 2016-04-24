%% FMCW Example -----------------------------------------------------------
% Based on Automotive Radar Example from Matlab
%   Copyright 2012-2015 The MathWorks, Inc.
clear
close all
tic


% System waveform parameters ---------------------------------------------
fc = 2.445e9;  
c = 3e8;   
rangeMax = 80;   
LPmixer = 28e3;
load('SampleRadiationPatterns.mat', 'TPLink'); % Antenna Model
rad_pat = TPLink; clear TPLink;

% Vehicle parameters ------------------------------------------------------
% Our system
Nsweep = 4;
rangeRes = 1;
bw = 70e6; %range2bw(rangeRes,c); % Increasing bandwidth improves resolution
% changing Tm does not change the power level/ SIR calculations
% but makes simulation faster
tm = 40e-3;      
radar_speed_x = 0;                    %m/s
radar_init_pos = [0;0;0.5];           %m
tgt_speed_x = 35/((tm/2)*(Nsweep-1));   %m/s, 
tgt_init_pos = [10;0;0.5];            %m

% Interferer waveform parameters -----------------------------------------
wave = 1;

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

% itferData = [x, y, dx, tm, bw] -100/((tm)*(Nsweep-1)/2)
itferData = [20, 3.048, -15/((tm)*(Nsweep-1)/2), tm_INT, bw_INT];
%             15, -3.048,0, tm//42, bw;
%               40, -3.048,0, tm/2, bw;
%                55, -3.048,0, tm/2, bw];
target = 'person';

% Antenna Model Set Up ---------------------------------------------------
% % MIT Values
txPower = 1; %db2pow(5)*1e-3;                     % in watts
txLossFactor = 0;                             % in dB **TODO**

% IF Power = -28 dBm
rxNF = 4.5;                                    % in dB
rxLossFactor = 0;                             % in dB **TODO
toc

% Turn on and off sections of code ---------------------------------------
PLOT.VEHICLES = 1;
PLOT.ACCURACY = 1;
PLOT.PREVIEW = 0;
PLOT.BEATSIGNAL = 1;
PLOT.CHIRP = 0;
PLOT.SIR = 1;
ONE_WAY_CHANNEL = 0;
SAVE = 0;
PHASE_SHIFT = 1;
MUTUAL_INTERFERENCE =1;
TARGET = 1;
fileName = 'filename.mat';


% Preview vehicle positions
% get the duration of the simulation
[radarPos, tgtPos, itferPos,...
    radarVel, tgtVel, itferVel] = prevEnv(Nsweep, tm,...
    radar_init_pos, tgt_init_pos, itferData,...
    radar_speed_x, tgt_speed_x, PLOT.PREVIEW, MUTUAL_INTERFERENCE, TARGET);

% If we included funcitonality for individual interferer waveforms
if size(itferData,2) > 3
    tm_INT = itferData(:,4,:);
    bw_INT = itferData(:,5,:);
end

%% Run the function
[output, beatsignal, fs_bs, signalRMS2, interfererRMS2, SIRdB] = radarSim(fc, tm, tm_INT, rangeMax, bw, bw_INT, Nsweep, LPmixer,...
    rad_pat, radarPos, itferPos, tgtPos, radarVel, itferVel, tgtVel,...
    txPower, txLossFactor,rxNF,...
    rxLossFactor,...
    PLOT, MUTUAL_INTERFERENCE, TARGET, ...
    PHASE_SHIFT, SAVE, fileName, target);
% 
clearvars -except beatsignal fs_bs output signalRMS2 interfererRMS2 SIRdB
% 
 %%
%  bs_t = plotBeatSignal(beatsignal, fs_bs,1, 1,1);
% % % load('scen23_wave1_10m.mat', 'beatsignal', 'fs_bs')
% % [output] = calcSimSIR(beatsignal, fs_bs)



