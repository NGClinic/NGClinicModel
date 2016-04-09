%% FMCW Example
% Based on Automotive Radar Example from Matlab
%   Copyright 2012-2015 The MathWorks, Inc.
clc
clear
close all

tic
%% Turn on and off sections of code --------------------------
PLOT.VEHICLES = 1;
PLOT.POWER = 0;
PLOT.ACCURACY = 1;
PLOT.CHIRP = 0;
PLOT.MUTUAL_INTERFERENCE_SPECTROGRAM = 0;
MUTUAL_INTERFERENCE= 1;
ONE_WAY_CHANNEL = 0;

% Scenario 1 == No interferer
% Scenario 2 == Interferer in opposing lane, no target objects
% Scenario 3 == Interferer in opposing lane, target;
% Scenario 4 == Direct interference


%% FMCWsimsetup --------------------------------------------------

%% System waveform parameters ----------------------------------------
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
Nsweep = 8;
n = 2.5e3;
fs_bs = fs/n;

%% Interferer waveform parameters ---------------------------------
bw_INT = bw; %40e6;
tm_INT = tm/2;


%% Vehicle placement -----------------------------------------------
% Our system
radar_speed = 1;    %m/s
radar_init_pos = [0;0;0.5]; %m
car_speed = 30; % m/s, 
car_init_pos = [5;0;0.5];   %m
itfer_speed = 0;
itfer_init_pos = [10, 2.7432, 0.5]';
[car_rng, ~] = rangeangle(car_init_pos, radar_init_pos);
[int_rng, int_ang] = rangeangle(itfer_init_pos, radar_init_pos);

%% FMCW Generation --------------------------------------------------
hwav = phased.FMCWWaveform('SweepTime',tm/2,'SweepBandwidth',bw,...
    'SampleRate',fs, 'SweepDirection', 'Triangle', 'NumSweeps', 2); %full triangle

hwav_INT = phased.FMCWWaveform('SweepTime',tm_INT/2,'SweepBandwidth',bw_INT,...
    'SampleRate',fs, 'SweepDirection', 'Triangle', 'NumSweeps', 4); %full triangle


%% Radar Parameters -----------------------------------------------
hradarplatform = phased.Platform('InitialPosition',radar_init_pos,...
    'Velocity',[radar_speed;0;0]);
hspec = dsp.SpectrumAnalyzer('SampleRate',fs,...
    'PlotAsTwoSidedSpectrum',true,...
    'Title','Spectrum for received and dechirped signal',...
    'ShowLegend',true);

%% Target Model Parameters --------------------------------------
car_rcs = db2pow(min(10*log10(car_rng)+5,20));
hcar = phased.RadarTarget('MeanRCS',car_rcs,'PropagationSpeed',c,...
    'OperatingFrequency',fc);
hcarplatform = phased.Platform('InitialPosition',...
    car_init_pos,...
    'Velocity',[car_speed;0;0]);

%% Interference Model ------------------------------------------

% Car lanes are about 10 ft --> 3.6576 m
itfer_rcs = db2pow(min(10*log10(int_rng)+5,20));
hitfer = phased.RadarTarget('MeanRCS',itfer_rcs,'PropagationSpeed',c,...
    'OperatingFrequency',fc);
hitferplatform = phased.Platform('InitialPosition',...
    itfer_init_pos,...
    'Velocity',[itfer_speed;0;0]);


%% Free Space Channel Set Up ----------------------------------
hchannel_twoway = phased.FreeSpace('PropagationSpeed',c,...
    'OperatingFrequency',fc,'SampleRate',fs,'TwoWayPropagation',true);
hchannel_oneway = phased.FreeSpace('PropagationSpeed',c,...
    'OperatingFrequency',fc,'SampleRate',fs,'TwoWayPropagation',false);


%% Antenna Model Set Up ----------------------------------------------
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

htx = phased.Transmitter('PeakPower',tx_ppower,...
    'Gain',tx_gain,...
    'LossFactor',tx_loss_factor);
hrx = phased.ReceiverPreamp('Gain',rx_gain,...
    'NoiseFigure',rx_nf,...
    'LossFactor', rx_loss_factor,...
    'SampleRate',fs);


%% Simulation Loop ---------------------------------------------------

%Initializing zero-vectors
radar_pos = zeros(Nsweep,3);
radar_vel = zeros(Nsweep,3);
tgt_pos = zeros(Nsweep,3);
tgt_vel = zeros(Nsweep, 3);
itfer_pos = zeros(Nsweep, 3);
itfer_vel = zeros(Nsweep,3);
maxdist = 10;
xr.NoINT = zeros(length(step(hwav))/n, Nsweep);
xr.INT = zeros(length(step(hwav))/n, Nsweep);
beatsignal.NoINT = zeros((tm/2)*fs_bs*2*Nsweep, 1);
toc
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
        signal.xtgt = step(hchannel_oneway,...
            signal.xrefl,tgt_pos(m,:)',radar_pos(m,:)',...
            tgt_vel(m,:)',radar_vel(m,:)');    % propagate through channel

    else
        signal.xp = step(hchannel_twoway,...
            signal.xt,...
            radar_pos(m,:)',....
            tgt_pos(m,:)',...
            radar_vel(m,:)',...
            tgt_vel(m,:)');                     % Propagate signal
        signal.xtgt = step(hcar,signal.xp);        % Reflect the signal
    end
       
    % Interfering Signal
    if MUTUAL_INTERFERENCE
        % Beat signal without inteference
        signal.xrx = step(hrx,signal.xtgt); % receive the signal
        xd = downsample(dechirp(signal.xrx,signal.x),n); % dechirp the signal
        xr.NoINT(:,m) = xd;                             % buffer the dechirped signal
        beatsignal.NoINT((((tm/2)*fs_bs*2)*(m-1)+1):((tm/2)*fs_bs*2*m)) = xd;
        
        % Beat signal with interference
        xitfer_gen = step(hwav_INT);                % Generate interfer signal
        xitfer_t = step(htx, xitfer_gen);       % Transmit interfer signal
        signal.xitfer = step(hchannel_oneway, xitfer_t, ...
            itfer_pos(m,:)', radar_pos(m,:)',...
            itfer_vel(m,:)', radar_vel(m,:)');  % Propagate through channel       
        signal.xrx = step(hrx,(signal.xtgt + signal.xitfer));  % receive the signal
        xd = downsample(dechirp(signal.xrx,signal.x),n); % dechirp the signal
        xr.INT(:,m) = xd;                             % buffer the dechirped signal
        beatsignal.INT((((tm/2)*fs_bs*2)*(m-1)+1):((tm/2)*fs_bs*2*m)) = xd;
        
    else
        signal.xrx = step(hrx,signal.xtgt); % receive the signal
        % Beat signal without interference
        xd = downsample(dechirp(signal.xrx,signal.x),n); % dechirp the signal
        xr.NoINT(:,m) = xd;                             % buffer the dechirped signal
        beatsignal.NoINT((((tm/2)*fs_bs*2)*(m-1)+1):((tm/2)*fs_bs*2*m)) = xd;
    end
   
    
end
clear xd
toc
%% Plot Beat Signal TD and FD


if MUTUAL_INTERFERENCE
    figure(2)
    subplot(211)
    dur = length(xr.INT(:,1))/fs_bs;
    bs_t = (0:(1/fs_bs):(((length(beatsignal.INT))/fs_bs) - (1/fs_bs)))*1000;
    plot(bs_t, abs(beatsignal.INT))
    title('Beat Signal with Interference')
    xlabel('time(ms)')
    ylabel('Amplitude')
    set(gca, 'xtick', [0:(dur - (1/fs_bs)):(((length(beatsignal.INT))/fs_bs) - (1/fs_bs))].*1000)
    grid on

    subplot(212)
    NFFT = 2^nextpow2(length(beatsignal.INT));
    f = (((-NFFT/2)):(NFFT/2-1))*(fs_bs/NFFT);
    plot(f, fftshift(abs(fft(beatsignal.INT, NFFT))))
    title('FFT')
    xlabel('freq (Hz)')
    ylabel('|fft|')
    xlim([-4000 4000])
end

figure(1)
subplot(211)
dur = length(xr.NoINT(:,1))/fs_bs;
bs_t = (0:(1/fs_bs):(((length(beatsignal.NoINT))/fs_bs) - (1/fs_bs)))*1000;
plot(bs_t, abs(beatsignal.NoINT))
title('Beat Signal without Interference')
xlabel('time(ms)')
ylabel('Amplitude')
% set(gca, 'xtick', [0:(dur - (1/fs_bs)):(((length(beatsignal.NoINT))/fs_bs) - (1/fs_bs))].*1000)
% set(gca, 'XTickLabel', num2str(get(gca, 'XTick')', '%1.1f'));
grid on

subplot(212)
NFFT = 2^nextpow2(length(beatsignal.NoINT));
f = (((-NFFT/2)):(NFFT/2-1))*(fs_bs/NFFT);
plot(f, fftshift(abs(fft(beatsignal.NoINT, NFFT))))
title('FFT')
xlabel('freq (Hz)')
ylabel('|fft|')
xlim([-4000 4000])


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



%% Process beat signal for calculations
xr_upsweep_NoINT = xr.NoINT(1:hwav.SweepTime*fs_bs,:);
xr_downsweep_NoINT = xr.NoINT((hwav.SweepTime*fs_bs):end, :);

if MUTUAL_INTERFERENCE
    xr_upsweep_INT = xr.INT(1:hwav.SweepTime*fs_bs,:);
    xr_downsweep_INT = xr.INT((hwav.SweepTime*fs_bs):end, :);
end

%% Calculation Range Distance
Ncalc = floor(Nsweep/4);


fbu_rng_NoINT = rootmusic(pulsint(xr_upsweep_NoINT,'coherent'),1,fs_bs);
fbd_rng_NoINT = rootmusic(pulsint(xr_downsweep_NoINT,'coherent'),1,fs_bs);
output.rng_est_NoINT = beat2range([fbu_rng_NoINT fbd_rng_NoINT],sweep_slope,c)/2;
fd_NoINT = -(fbu_rng_NoINT+fbd_rng_NoINT)/2;
output.v_est_NoINT = dop2speed(fd_NoINT,lambda)/2;

rng_est_NoINT = zeros(Nsweep,1);
rng_true_NoINT = zeros(Nsweep,1);
v_est_NoINT = zeros(Nsweep, 1);
for i = 1:Nsweep
    fbu_rng_NoINT = rootmusic(pulsint(xr_upsweep_NoINT(:,i),'coherent'),1,fs_bs);
    fbd_rng_NoINT = rootmusic(pulsint(xr_downsweep_NoINT(:,i),'coherent'),1,fs_bs);
    rng_est_NoINT(i) = beat2range([fbu_rng_NoINT fbd_rng_NoINT],sweep_slope,c)/2;
    fd_NoINT = -(fbu_rng_NoINT+fbd_rng_NoINT)/2;
    v_est_NoINT(i) = dop2speed(fd_NoINT,lambda)/2;
    rng_true_NoINT(i) = sqrt(sum((radar_pos(i,:)-tgt_pos(i,:)).^2));
end

if MUTUAL_INTERFERENCE
    fbu_rng_INT = rootmusic(pulsint(xr_upsweep_INT,'coherent'),1,fs_bs);
    fbd_rng_INT = rootmusic(pulsint(xr_downsweep_INT,'coherent'),1,fs_bs);
    output.rng_est_INT = beat2range([fbu_rng_INT fbd_rng_INT],sweep_slope,c)/2;
    fd_INT = -(fbu_rng_INT+fbd_rng_INT)/2;
    output.v_est_INT = dop2speed(fd_INT,lambda)/2;

    rng_est_INT = zeros(Nsweep,1);
    v_est_INT = zeros(Nsweep, 1);
    for i = 1:Nsweep
        fbu_rng_INT = rootmusic(pulsint(xr_upsweep_INT(:,i),'coherent'),1,fs_bs);
        fbd_rng_INT = rootmusic(pulsint(xr_downsweep_INT(:,i),'coherent'),1,fs_bs);
        rng_est_INT(i) = beat2range([fbu_rng_INT fbd_rng_INT],sweep_slope,c)/2;
        fd_INT = -(fbu_rng_INT+fbd_rng_INT)/2;
        v_est_INT(i) = dop2speed(fd_INT,lambda)/2;
    end
end

%% Plot accuracy of calculations

if (PLOT.ACCURACY)
    figure
    subplot(211);
    grid on
    hold on
    suptitle(['Accuracy with ' num2str(Nsweep) ' Sweeps'])
    t = (0:Nsweep-1)*hwav.SweepTime*hwav.NumSweeps;
    plot(t, rng_true_NoINT, '.-', 'DisplayName', 'Target Range (m)')
    plot(t, rng_est_NoINT,'.-', 'DisplayName', 'Calc w/o Int (m)'); 
    if MUTUAL_INTERFERENCE
        plot(t, rng_est_INT, '.-', 'DisplayName', 'Calc w/Int (m) ')
    end
    legend('Location', 'eastoutside'); 
    hold off
    xlim([0, Nsweep*hwav.SweepTime*hwav.NumSweeps])
    title('Range'); ylabel('m'); xlabel('s');
   
    subplot(212);
    grid on
    hold on
    plot(t, -(radar_speed-car_speed)*ones(Nsweep,1), ...
        '.-', 'DisplayName', 'Target Speed (m/s)')
    plot(t, -v_est_NoINT, '.-', 'DisplayName', 'Calc w/o Int (m/s)');
    if MUTUAL_INTERFERENCE
        plot(t, -v_est_INT, '.-', 'DisplayName', 'Calc w/Int (m/s)');
    end
    hold off
    legend('Location', 'eastoutside'); 
    xlim([0, Nsweep*hwav.SweepTime*hwav.NumSweeps])
    title('Velocity'); ylabel('m/s');  xlabel('s');
    
     
end
toc