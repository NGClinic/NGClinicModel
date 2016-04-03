%% FMCW Example
% Based on Automotive Radar Example from Matlab
%   Copyright 2012-2015 The MathWorks, Inc.
clc
close all
clear

tic
%% Turn on and off sections of code
PLOT.VEHICLES = 1;
PLOT.POWER = 0;
PLOT.ACCURACY = 1;
PLOT.CHIRP = 0;
PLOT.MUTUAL_INTERFERENCE_SPECTROGRAM = 0;
SCENARIO = 1;
MUTUAL_INTERFERENCE= 0;
ONE_WAY_CHANNEL = 0;

% Scenario 1 == No interferer
% Scenario 2 == Interferer in opposing lane, no target objects
% Scenario 3 == Interferer in opposing lane, target;
% Scenario 4 == Direct interference


%% FMCWsimsetup

% Constants
fc = 2.43e9;  
c = 3e8;   
lambda = c/fc;  
range_max = 80;   
tm = 20e-3; 
range_res = 1;  
bw = 70e6; %range2bw(range_res,c);
sweep_slope = bw/tm;        
fr_max = range2beat(range_max,sweep_slope,c); 
v_max = 230*1000/3600; 
fd_max = speed2dop(2*v_max,lambda);
fb_max = fr_max+fd_max;
fs = max(2*fb_max,bw);

% Vehicle placement

% Our system
radar_speed = 1;    %m/s, 60mph
radar_init_pos = [0;0;0.5];
car_speed = 26.82; % m/s, 70 mph
car_init_pos = [5;0;0.5]'
itfer_speed = 1;
itfer_init_pos = [10, 2.7432, 0.5]';
[int_rng, int_ang] = rangeangle(itfer_init_pos, radar_init_pos);

toc
%% FMCW Generation
tic
hwav = phased.FMCWWaveform('SweepTime',tm/2,'SweepBandwidth',bw,...
    'SampleRate',fs, 'SweepDirection', 'Triangle', 'NumSweeps', 2); %full triangle


%% Radar Parameters
hradarplatform = phased.Platform('InitialPosition',radar_init_pos,...
    'Velocity',[radar_speed;0;0]);
hspec = dsp.SpectrumAnalyzer('SampleRate',fs,...
    'PlotAsTwoSidedSpectrum',true,...
    'Title','Spectrum for received and dechirped signal',...
    'ShowLegend',true);

%% Target Model Parameters
car_rcs = db2pow(min(10*log10(car_dist)+5,20));
hcar = phased.RadarTarget('MeanRCS',car_rcs,'PropagationSpeed',c,...
    'OperatingFrequency',fc);
hcarplatform = phased.Platform('InitialPosition',...
    car_init_pos,...
    'Velocity',[car_speed;0;0]);

%% Interference Model

% Car lanes are about 10 ft --> 3.6576 m
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
ant_gain = 9;  % value from MIT Slide deck || 10*log10((pi*ant_dia/lambda)^2);

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
toc
%% Simulation Loop 
tic
Nsweep = 16;


%% Initializing zero-vectors
radar_pos = zeros(Nsweep,3);
radar_vel = zeros(Nsweep,3);
tgt_pos = zeros(Nsweep,3);
tgt_vel = zeros(Nsweep, 3);
itfer_pos = zeros(Nsweep, 3);
itfer_vel = zeros(Nsweep,3);
xr = zeros(length(step(hwav)), Nsweep);
maxdist = 10;
beatsignal = zeros((tm/2)*fs*2*Nsweep, 1);

% radar_pos(:,1) = (1:Nsweep).*hwav.SweepTime.*radar_speed(1) + radar_init_pos(1);
% radar_pos(:,2) = (1:Nsweep).*hwav.SweepTime.*radar_speed(2) + radar_init_pos(2);
% radar_pos(:,3) = (1:Nsweep).*hwav.SweepTime.*radar_speed(3) + radar_init_pos(3);

% radar_pos = repmat(1:Nsweep,3,1)'.*hwav.SweepTime.*repmat(radar_speed',3,1)+ repmat(radar_init_pos',3,1);
% Simulation for multiple Sweeps
toc
tic
for m = 1:Nsweep
    
    % Move objects
    [radar_pos(m,:),radar_vel(m,:)] = step(...
        hradarplatform,hwav.SweepTime*hwav.NumSweeps);   % radar moves during sweep
    [tgt_pos(m,:),tgt_vel(m,:)] = step(hcarplatform,... 
        hwav.SweepTime*hwav.NumSweeps);                  % car moves during sweep
    [itfer_pos(m,:), itfer_vel(m,:)] = step(hitferplatform,...
        hwav.SweepTime*hwav.NumSweeps);                  % interferer moves during sweep
  
    % Generate Our Signal
    signal.x = step(hwav);                      % generate the FMCW signal
    signal.xt = step(htx,signal.x);             % transmit the signal
       
    if ONE_WAY_CHANNEL
        signal.xp = step(hchannel_oneway,signal.xt,radar_pos(m,:)',...
             tgt_pos(m,:)',...
             radar_vel(m,:)',...
             tgt_vel(m,:)');                   % propagate through channel
        signal.xrefl = step(hcar,signal.xp);                 % reflect the signal 
        signal.xdone = step(hchannel_oneway,...
            signal.xrefl,tgt_pos(m,:)',radar_pos(m,:)',...
            tgt_vel(m,:)',radar_vel(m,:)');    % propagate through channel

    else
        signal.xp = step(hchannel_twoway,...
            signal.xt,...
            radar_pos(m,:)',....
            tgt_pos(m,:)',...
            radar_vel(m,:)',...
            tgt_vel(m,:)');                     % Propagate signal
        signal.xdone = step(hcar,signal.xp);        % Reflect the signal
    end
    
   
    % Interfering Signal
    if MUTUAL_INTERFERENCE
        xitfer_gen = step(hwav);                % Generate interfer signal
        xitfer_t = step(htx, xitfer_gen);       % Transmit interfer signal
        signal.xitfer = step(hchannel_oneway, xitfer_t, ...
            itfer_pos(m,:)', radar_pos(m,:)',...
            itfer_vel(m,:)', radar_vel(m,:)');  % Propagate through channel       
        signal.xrx = step(hrx,(signal.xdone + signal.xitfer));                        % receive the signal
    else
        signal.xrx = step(hrx,signal.xdone);
    end
     
    xd = dechirp(signal.xrx,signal.x);           % dechirp the signal
    xr(:,m) = xd;                             % buffer the dechirped signal
    beatsignal((((tm/2)*fs*2)*(m-1)+1):((tm/2)*fs*2*m)) = xd;
end

toc
%%
% figure 
% bs_t = 0:(1/fs):(((length(beatsignal))/fs) - (1/fs));
% plot(bs_t, abs(beatsignal))
% title('Beat Signal')
% xlabel('time(s)')
% ylabel('Amplitude')
% grid on
% dur = length(xd)/fs;
% set(gca, 'xtick', [0:(dur - (1/fs)):(((length(beatsignal))/fs) - (1/fs))])
% set(gca,'XTick',bs_t)
% set(gca,'XTickLabel',sprintf('%1.2f|',bs_t))
%%
dur = length(xd)/fs;
xd_t = 0:(1/fs):(dur - (1/fs));
figure
hold on
for i=1:Nsweep
    plot(xd_t, abs(xr(:,i)),'DisplayName', num2str(i))
end
legend('Location', 'eastoutside')
title('Beat Signal')
xlabel('time(s)')
ylabel('amp')

%%
figure
imagesc(1:32, xd_t,abs(xr));
xlabel('Sweep Interval')
ylabel('time (s)')
title('Beat Signal')
set(gca,'YDir','normal')
colorbar

%% Plot Spectrogram
if PLOT.MUTUAL_INTERFERENCE_SPECTROGRAM
    mult = 2^4;
    figure
    subplot(211)
    spectrogram(step(hrx, xdone), 32*mult, 16*mult, 32*mult, fs, 'yaxis')
    title('No Mutual Interference')
    subplot(212)
    spectrogram(xrx, 32*mult, 16*mult, 32*mult, fs,'yaxis')
    title('With Mutual Interference')
    suptitle('Spectrogram Interference Effects')
end 


%% Plotting Spectral Density
if (PLOT.POWER)
    field = fieldnames(signal);
    figure
    for n=1:length(field)
        figure
        x = signal.(field{n});
        [px,f] = periodogram(x, 2*hamming(length(x)), 2^nextpow2(length(x)), fs);
        px = 10*log10(px);
        plot(f,px,'-','DisplayName', ['(' num2str(n) ') '  field{n}]);
        title('Periodogram Power Spectral Density Estimate')
        legend('Location', 'eastoutside')
        text(f(6), px(6), num2str(n))   
        xlabel('Frequency (Hz)')
        ylabel('Power (dB)')
    end
    hold off
    title('Periodogram Power Spectral Density Estimate')
    legend('Location', 'eastoutside')
    
end

clear itfer_por itfer_speed 

%% Plotting Vehicle Positions
if (PLOT.VEHICLES)
    figure
    radar_pos_x = radar_pos(:,1);
    radar_pos_y = radar_pos(:,2);
    radar_pos_z = radar_pos(:,3);
    tgt_pos_x = tgt_pos(:,1);
    tgt_pos_y = tgt_pos(:,2);
    tgt_pos_z = tgt_pos(:,3);
    int_pos_x = itfer_pos(:,1);
    int_pos_y = itfer_pos(:,2);
    int_pos_z = itfer_pos(:,3);
    hold on
    plot(radar_pos_x,radar_pos_y, 'g-', 'DisplayName','Our Radar');
    plot(radar_pos_x(1),radar_pos_y(1), 'go', 'DisplayName', 'Start');
    plot(radar_pos_x(Nsweep),radar_pos_y(Nsweep), 'gx', 'DisplayName', 'End');

    plot(tgt_pos_x, tgt_pos_y, 'k-', 'DisplayName', 'Target System');
    plot(tgt_pos_x(1), tgt_pos_y(1), 'ko', 'DisplayName', 'Start');
    plot(tgt_pos_x(Nsweep), tgt_pos_y(Nsweep), 'kx', 'DisplayName', 'End');
    if (MUTUAL_INTERFERENCE)
        plot(int_pos_x, int_pos_y, 'r-', 'DisplayName', 'Interferer System');
        plot(int_pos_x(1), int_pos_y(1), 'ro', 'DisplayName', 'Start');
        plot(int_pos_x(Nsweep), int_pos_y(Nsweep), 'rx', 'DisplayName', 'End');
    end
    xlabel('X (m)')
    ylabel('Y (m)')
    legend('Location', 'eastoutside')
    title('Position of Vehicles')
    grid
    hold off
end


%% Calculate Range and Doppler
% TODO Fix

xr_upsweep = xr(1:hwav.SweepTime*fs,:);
xr_downsweep = xr((hwav.SweepTime*fs):end, :);
% if (1)
%     figure
%     hrdresp = phased.RangeDopplerResponse('PropagationSpeed',c,...
%         'DopplerOutput','Speed',...
%         'OperatingFrequency',fc,...
%         'SampleRate',fs,...
%         'RangeMethod','FFT',...
%         'DechirpInput', false,...
%         'SweepSlope',sweep_slope,...
%         'RangeFFTLengthSource','Property','RangeFFTLength',2048,...
%         'DopplerFFTLengthSource','Property','DopplerFFTLength',256);
% 
%     clf;
%     plotResponse(hrdresp,xr_upsweep)                    % Plot range Doppler map
%   %  axis([-v_max v_max 0 range_max])
%     clim = caxis;
% end

%% Calculation Range Distance
Ncalc = floor(Nsweep/4);


fbu_rng = rootmusic(pulsint(xr_upsweep,'coherent'),1,fs);
fbd_rng = rootmusic(pulsint(xr_downsweep,'coherent'),1,fs);
output.rng_est = beat2range([fbu_rng fbd_rng],sweep_slope,c)/2;
fd = -(fbu_rng+fbd_rng)/2;
output.v_est = dop2speed(fd,lambda)/2;

rng_est = zeros(Nsweep,1);
rng_true = zeros(Nsweep,1);
v_est = zeros(Nsweep, 1);
for i = 1:Nsweep
    fbu_rng = rootmusic(pulsint(xr_upsweep(:,i),'coherent'),1,fs);
    fbd_rng = rootmusic(pulsint(xr_downsweep(:,i),'coherent'),1,fs);
    rng_est(i) = beat2range([fbu_rng fbd_rng],sweep_slope,c)/2;
    fd = -(fbu_rng+fbd_rng)/2;
    v_est(i) = dop2speed(fd,lambda)/2;
    rng_true(i) = sqrt(sum((radar_pos(i,:)-tgt_pos(i,:)).^2));
end

%% Plot accuracy of calculations

if (PLOT.ACCURACY)
    figure
    subplot(211);
    suptitle(['Accuracy with ' num2str(Nsweep) ' Sweeps'])
    t = (1:Nsweep)*hwav.SweepTime*hwav.NumSweeps;
    plot(t, rng_true, '.-', 'DisplayName', 'Target Range (m)')
    hold on
    plot(t, rng_est,'.-', 'DisplayName', 'Calculated Range (m)');
    legend('Location', 'eastoutside'); title('Range'); ylabel('m'); xlabel('s');
%     axis([0 0.08 8 11.1])
    xlim([0, Nsweep*hwav.SweepTime*hwav.NumSweeps])

   
    subplot(212);
    plot(t, -(radar_speed-car_speed)*ones(Nsweep,1), ...
        '.-', 'DisplayName', 'Target Speed (m/s)')
    hold on
    plot(t, -v_est, '.-', 'DisplayName', 'Calculated Speed (m/s)');
    hold off
    legend('Location', 'eastoutside')
    xlim([0, Nsweep*hwav.SweepTime*hwav.NumSweeps])
    title('Velocity'); ylabel('m/s');  xlabel('s');
end