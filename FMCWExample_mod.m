close all
clc
clear

%%% Turn on and off sections of code
PLOT.VEHICLES = 1;
PLOT.POWER = 0;
PLOT.CALCULATIONS = 1;
PLOT.CHIRP = 0;
PLOT.MUTUAL_INTERFERENCE_SPECTROGRAM = 0;
MUTUAL_INTERFERENCE = 0;
%NUM_INTERFERERS
ONE_WAY_CHANNEL = 1;    % Set 0 if two way channel. 
                        % Haven't tested two-way channel.
                        % Algorithmically should be fine. Variable name
                        % could be buggy.

%% FMCW Example
% Based on Automotive Radar Example from Matlab
%   Copyright 2012-2015 The MathWorks, Inc.

%% Constants
fc = 77e9;
c = 3e8;
lambda = c/fc;
range_max = 200;
tm = 1.25e-3; %5.5*range2time(range_max,c);
range_res = 1;
bw = range2bw(range_res,c);
sweep_slope = bw/tm;
fr_max = range2beat(range_max,sweep_slope,c);
v_max = 230*1000/3600;
fd_max = speed2dop(2*v_max,lambda);
fb_max = fr_max+fd_max;
fs = max(2*fb_max,bw);

%% FMCW Generation
hwav = phased.FMCWWaveform('SweepTime',tm/2,'SweepBandwidth',bw,...
    'SampleRate',fs, 'SweepDirection', 'Triangle', 'NumSweeps', 2);

if (PLOT.CHIRP)
    s = step(hwav);
    subplot(211); 
    t = (1:length(s))*1/fs;
    plot(t,real(s));
    xlabel('Time (s)'); ylabel('Amplitude (v)');
    title('FMCW signal'); axis tight;
    subplot(212); spectrogram(s,32,16,32,fs,'yaxis');
    title('FMCW signal spectrogram');
end

%% Radar Parameters
radar_speed = 20;
hradarplatform = phased.Platform('InitialPosition',[0;0;0.5],...
    'Velocity',[radar_speed;0;0]);
hspec = dsp.SpectrumAnalyzer('SampleRate',fs,...
    'PlotAsTwoSidedSpectrum',true,...
    'Title','Spectrum for received and dechirped signal',...
    'ShowLegend',true);

%% Target Model Parameters
car_dist = 5;
car_speed = 30;
car_rcs = db2pow(min(10*log10(car_dist)+5,20));
hcar = phased.RadarTarget('MeanRCS',car_rcs,'PropagationSpeed',c,...
    'OperatingFrequency',fc);
hcarplatform = phased.Platform('InitialPosition',...
    [hradarplatform.InitialPosition(1)+car_dist;0;0.5],...
    'Velocity',[car_speed;0;0]);

%% Interference Model
itfer_init_pos = [hcarplatform.InitialPosition(1)+200, 0, 2.5]';
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
ant_aperture = 6.06e-4;                         % in square meter
ant_gain = aperture2gain(ant_aperture,lambda);  % in dB

tx_ppower = db2pow(5)*1e-3;                     % in watts
tx_gain = 9+ant_gain;                           % in dB
tx_loss_factor = 0;                             % in dB **TODO**

rx_gain = 15+ant_gain;                          % in dB
rx_nf = 4.5;                                    % in dB
rx_loss_factor = 0;                             % in dV **TODO

htx = phased.Transmitter('PeakPower',tx_ppower,...
    'Gain',tx_gain,...
    'LossFactor',tx_loss_factor);
hrx = phased.ReceiverPreamp('Gain',rx_gain,...
    'NoiseFigure',rx_nf,...
    'LossFactor', rx_loss_factor,...
    'SampleRate',fs);

%% Simulation Loop 
N = 64;
Nsweep = 64;
envSNR = 10;
xr = complex(zeros(hwav.SampleRate*hwav.SweepTime*hwav.NumSweeps,Nsweep));
radar_pos = zeros(Nsweep,3);
radar_vel = zeros(Nsweep,3);
tgt_pos = zeros(Nsweep,3);
tgt_vel = zeros(Nsweep, 3);
itfer_pos = zeros(Nsweep, 3);
itfer_vel = zeros(Nsweep,3);

for m = 1:Nsweep
    
    % Move objects
    [radar_pos(m,:),radar_vel(m,:)] = step(...
        hradarplatform,hwav.SweepTime*hwav.NumSweeps);   % radar moves during sweep
    [tgt_pos(m,:),tgt_vel(m,:)] = step(hcarplatform,... 
        hwav.SweepTime*hwav.NumSweeps);              % car moves during sweep
    [itfer_pos(m,:), itfer_vel(m,:)] = step(hitferplatform,...
        hwav.SweepTime*hwav.NumSweeps);              % interferer moves during sweep

    
    % Generate Our Signal
    x = step(hwav);                           % generate the FMCW signal
    xt = step(htx,x);                         % transmit the signal
       
    if ONE_WAY_CHANNEL
        xp = step(hchannel_oneway,xt,radar_pos(m,:)',...
             tgt_pos(m,:)',...
             radar_vel(m,:)',...
             tgt_vel(m,:)');                   % propagate through channel
        xrefl = step(hcar,xp);                 % reflect the signal 
        xdone = step(hchannel_oneway,...
            xrefl,tgt_pos(m,:)',radar_pos(m,:)',...
            radar_vel(m,:)',tgt_vel(m,:)');    % propagate through channel
        xdoneNoise = awgn(xdone,envSNR);
    else
        xp = step(hchannel_twoway,xt,radar_pos(m,:)',....
            tgt_pos(m,:)',...
            radar_vel(m,:)',tgt_vel(m,:)');  
        xdone = step(hcar,xp);                 % reflect the signal
    end        
     
    % Interfering Signal
    if MUTUAL_INTERFERENCE
        xitfer = step(hwav);                % Generate interfer signal
        xitfer_t = step(htx, xitfer);       % Transmit interfer signal
        xitfer_done = step(hchannel_oneway, xitfer_t, ...
            itfer_pos(m,:)', radar_pos(m,:)',...
            itfer_vel(m,:)', radar_vel(m,:)');  % Propagate through channel
    else
        xitfer_done = 0;
    end
     
    xrx = step(hrx,(xdone + xitfer_done));                        % receive the signal
    xd = dechirp(xrx,x);                       % dechirp the signal
    xr(:,m) = xd;                             % buffer the dechirped signal
end

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

%% Organizing Simulation Output Data
xr_upsweep = xr(1:hwav.SweepTime*fs,:);
xr_downsweep = xr((hwav.SweepTime*fs):end, :);

% Save data for ploting
[pxx,f] = periodogram(x, hamming(length(x)), nextpow2(length(x)), fs);
[pxxt,ft] = periodogram(xt, hamming(length(xt)), nextpow2(length(xt)), fs);    
[pxxp,fr] = periodogram(xp, hamming(length(xp)), nextpow2(length(xp)), fs);  
[pxxdone,fdone] = periodogram(xdone, hamming(length(xdone)), nextpow2(length(xdone)), fs);    
[pxxrx,frx] = periodogram(xrx, hamming(length(xrx)), nextpow2(length(xrx)), fs);

pxx = 10*log10(pxx);
pxxt = 10*log10(pxxt);
pxxp = 10*log10(pxxp);
pxxdone = 10*log10(pxxdone);
pxxrx = 10*log10(pxxrx);

if ONE_WAY_CHANNEL
    [pxxrefl,frefl] = periodogram(xrefl, hamming(length(xrefl)), nextpow2(length(xrefl)), fs);    
    pxxrefl = 10*log10(pxxrefl);
end

%% Plotting Power
if (PLOT.POWER)
    if ONE_WAY_CHANNEL
        figure
        plot(f, pxx,...
            ft,pxxt, ...
            fr, pxxp, ...
            frefl, pxxrefl,... 
            fdone, pxxdone,...
            frx, pxxrx)
        text(f(2), pxx(2),'(1)', 'BackgroundColor', 'w');
        text(f(2), pxxt(2),'(2)', 'BackgroundColor', 'w');
        text(f(2), pxxp(2),'(3)', 'BackgroundColor', 'w');
        text(f(2), pxxrefl(2),'(4)', 'BackgroundColor', 'w');
        text(f(2), pxxdone(2),'(5)', 'BackgroundColor', 'w');
        text(f(2), pxxrx(2),'(6)', 'BackgroundColor', 'w');


        title('Periodogram Power Spectral Density Estimate')
        legend('(1) Generated Signal', ...
            '(2) Transmitted Signal',...
            '(3) Propagated Signal',...
            '(4) Reflected Signal', ...  
            '(5) Return Propagation Signal',...
            '(6) Received Signal');
        xlabel('Frequency (Hz)')
        ylabel('Power (dB)')
    else
        figure
        plot(f, pxx,...
            ft,pxxt, ...
            fr, pxxp, ...
            fdone, pxxdone,...
            frx, pxxrx)
        text(f(2), pxx(2),'(1)', 'BackgroundColor', 'w');
        text(f(2), pxxt(2),'(2)', 'BackgroundColor', 'w');
        text(f(2), pxxp(2),'(3)', 'BackgroundColor', 'w');
        text(f(2), pxxdone(2),'(4)', 'BackgroundColor', 'w');
        text(f(2), pxxrx(2),'(5)', 'BackgroundColor', 'w');


        title('Periodogram Power Spectral Density Estimate')
        legend('(1) Generated Signal', ...
            '(2) Transmitted Signal',...
            '(3) Propagated Signal',...
            '(4) Reflected Signal', ...  
            '(5) Received Signal');
        xlabel('Frequency (Hz)')
        ylabel('Power (dB)')
    end
end

%% Plotting Target Positions
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
    plot(radar_pos_x,radar_pos_z, 'b-');
    plot(radar_pos_x(1),radar_pos_z(1), 'bo');
    plot(radar_pos_x(Nsweep),radar_pos_z(Nsweep), 'bx');

    plot(tgt_pos_x, tgt_pos_z, 'g-');
    plot(tgt_pos_x(1), tgt_pos_z(1), 'go');
    plot(tgt_pos_x(Nsweep), tgt_pos_z(Nsweep), 'gx');

    plot(int_pos_x, int_pos_z, 'r-');
    plot(int_pos_x(1), int_pos_z(1), 'ro');
    plot(int_pos_x(Nsweep), int_pos_z(Nsweep), 'rx');
    xlabel('X (m)')
    ylabel('Z (m)')
    legend('Our Radar','Our Start', 'Our End','Target System', ...
        'Target Start', 'Target End', 'Interfer', 'Interferer Start', 'Interferer Stop')
    title('Position of Vehicles')
    grid
    axis([-10 10 -10 10])
    hold off
end

%% Range and Doppler Estimation
% TODO Fix
if (0)
    figure
    hrdresp = phased.RangeDopplerResponse('PropagationSpeed',c,...
        'DopplerOutput','Speed',...
        'OperatingFrequency',fc,...
        'SampleRate',fs,...
        'RangeMethod','FFT',...
        'DechirpInput', false,...
        'SweepSlope',sweep_slope,...
        'RangeFFTLengthSource','Property','RangeFFTLength',2048,...
        'DopplerFFTLengthSource','Property','DopplerFFTLength',256);

    clf;
    plotResponse(hrdresp,xr_upsweep)                    % Plot range Doppler map
  %  axis([-v_max v_max 0 range_max])
    clim = caxis;
end

%% Calculation the range and distance
% Ncalc = floor(Nsweep/4);

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

%% Plotting
if (PLOT.CALCULATIONS)
    figure
    subplot(211);
    suptitle('Accuracy with 64 Sweeps, Tm = 2.5ms')
    plot((1:Nsweep)*hwav.SweepTime*hwav.NumSweeps, rng_true, ...
        (1:Nsweep)*hwav.SweepTime*hwav.NumSweeps, rng_est)
    legend('True Range (m)', 'Calculated Range (m)')
    title('Range')
    ylabel('m');
    xlabel('s');
    subplot(212);
    plot((1:Nsweep)*hwav.SweepTime*hwav.NumSweeps, (radar_speed-car_speed)*ones(Nsweep,1), ...
        (1:Nsweep)*hwav.SweepTime*hwav.NumSweeps, v_est);
    legend('True Speed (m)', 'Calculated Speed (m)')
    title('Velocity')
    ylabel('m/s');
    xlabel('s');
end

