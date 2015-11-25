%% Turn on and off sections of code
PLOT.VEHICLES = 1;
PLOT.POWER = 1;
PLOT.ACCURACY = 1;
PLOT.CHIRP = 1;
PLOT.MUTUAL_INTERFERENCE_SPECTROGRAM = 0;
MUTUAL_INTERFERENCE = 0;
ONE_WAY_CHANNEL = 1;

%% Constants
fc = 77e9;  % Same
c = 3e8;    % Same
lambda = c/fc;  % same
range_max = 200;    % 200-> 400
tm = 1.25e-3; %5.5*range2time(range_max,c);   %same
range_res = 1;  % same
bw = range2bw(range_res,c);     %same
sweep_slope = bw/tm;        %same
fr_max = range2beat(range_max,sweep_slope,c);       %same
v_max = 230*1000/3600;      %same
fd_max = speed2dop(2*v_max,lambda); %same
fb_max = fr_max+fd_max;     %same
fs = max(2*fb_max,bw);  %same

%% FMCW Generation
hwav = phased.FMCWWaveform('SweepTime',tm/2,'SweepBandwidth',bw,...
    'SampleRate',fs, 'SweepDirection', 'Triangle', 'NumSweeps', 2); %full triangle


%% Radar Parameters
radar_speed = 20; %40;    %m/s, 60mph
radar_init_pos = [0;0;0.5];
hradarplatform = phased.Platform('InitialPosition',radar_init_pos,...
    'Velocity',[radar_speed;0;0]);
hspec = dsp.SpectrumAnalyzer('SampleRate',fs,...
    'PlotAsTwoSidedSpectrum',true,...
    'Title','Spectrum for received and dechirped signal',...
    'ShowLegend',true);

%% Target Model Parameters
car_speed = 31.29; % m/s, 70 mph
car_dist = 60; %radar_speed*3;     %cars should be 3 seconds away!
car_rcs = db2pow(min(10*log10(car_dist)+5,20));
hcar = phased.RadarTarget('MeanRCS',car_rcs,'PropagationSpeed',c,...
    'OperatingFrequency',fc);
hcarplatform = phased.Platform('InitialPosition',...
    [hradarplatform.InitialPosition(1)+car_dist;0;0.5],...
    'Velocity',[car_speed;0;0]);

%% Interference Model
% Car lanes are about 10 ft --> 3.6576 m
itfer_init_pos = [hcarplatform.InitialPosition(1)+10, 3.6576, 0.5]';
itfer_speed = -30;
[int_rng, int_ang] = rangeangle(itfer_init_pos, hradarplatform.InitialPosition);
itfer_rcs = db2pow(min(10*log10(int_rng)+5,20));

hitfer = phased.RadarTarget('MeanRCS',itfer_rcs,'PropagationSpeed',c,...
    'OperatingFrequency',fc);
hitferplatform = phased.Platform('InitialPosition',...
    itfer_init_pos,...
    'Velocity',[itfer_speed;0;0]);

%% Free Space Channel Set Up
hchannel_twoway = phased.FreeSpace('PropagationSpeed',c,...
    'OperatingFrequency',fc,'SampleRate',fs,'TwoWayPropagation',true);
hchannel_oneway = phased.FreeSpace('PropagationSpeed',c,...
    'OperatingFrequency',fc,'SampleRate',fs,'TwoWayPropagation',false);

%% Antenna Model Set Up
% % MIT Values
ant_dia = 0.1524;   % coffee can is 6 inch. 0.1524 m.
ant_aperture = 6.06e-4; %pi*ant_dia^2;    %6.06e-4;       % in square meter
ant_gain = 8.4;  % value from MIT Slide deck || 10*log10((pi*ant_dia/lambda)^2);

tx_ppower = 0.65; %db2pow(5)*1e-3;                     % in watts
tx_gain = 24;   %9+ant_gain;                           % in dB
tx_loss_factor = 0;                             % in dB **TODO**

rx_power = 1;   %Watt
% IF Power = -28 dBm
rx_gain = 15+ant_gain;                          % in dB
rx_nf = 4.5;                                    % in dB
rx_loss_factor = 0;                             % in dB **TODO


% Original Example Values
% ant_aperture = 6.06e-4;                         % in square meter
% ant_gain = aperture2gain(ant_aperture,lambda);  % in dB                                                 %radiator
% 
% tx_ppower = db2pow(5)*1e-3;                     % in watts
% tx_gain = 9+ant_gain;                           % in dB
% tx_loss_factor = 0;      
% 
% rx_gain = 15+ant_gain;                          % in dB
% rx_nf = 4.5;                                    % in dB
% rx_loss_factor = 0;                             % in dB **TODO


htx = phased.Transmitter('PeakPower',tx_ppower,...
    'Gain',tx_gain,...
    'LossFactor',tx_loss_factor);
hrx = phased.ReceiverPreamp('Gain',rx_gain,...
    'NoiseFigure',rx_nf,...
    'LossFactor', rx_loss_factor,...
    'SampleRate',fs);