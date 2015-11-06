close all
clc
clear
%% FMCW Example
% Based on Automotive Radar Example from Matlab
%   Copyright 2012-2015 The MathWorks, Inc.

fc = 77e9;
c = 3e8;
lambda = c/fc;
range_max = 200;
tm = 2e-3; %5.5*range2time(range_max,c);
range_res = 1;
bw = range2bw(range_res,c);
sweep_slope = bw/tm;
fr_max = range2beat(range_max,sweep_slope,c);
v_max = 230*1000/3600;
fd_max = speed2dop(2*v_max,lambda);
fb_max = fr_max+fd_max;
fs = max(2*fb_max,bw);


%% FMCW Generation
hwav = phased.FMCWWaveform('SweepTime',tm,'SweepBandwidth',bw,...
    'SampleRate',fs, 'SweepDirection', 'Triangle', 'NumSweeps', 1);
hwav.SweepDirection = 'Triangle';

s_up = step(hwav);
s_down = step(hwav);
s = [s_up
     s_down];
subplot(211); 
t = (1:length(s))*1/fs;
plot(t,real(s));
xlabel('Time (s)'); ylabel('Amplitude (v)');
title('FMCW signal'); axis tight;
subplot(212); spectrogram(s,32,16,32,fs,'yaxis');
title('FMCW signal spectrogram');


%% Radar Parameters
radar_speed = 20;
hradarplatform = phased.Platform('InitialPosition',[0;0;0.5],...
    'Velocity',[radar_speed;0;0]);

hspec = dsp.SpectrumAnalyzer('SampleRate',fs,...
    'PlotAsTwoSidedSpectrum',true,...
    'Title','Spectrum for received and dechirped signal',...
    'ShowLegend',true);

%% Target Model
car_dist = 5;
car_speed = 30;
car_rcs = db2pow(min(10*log10(car_dist)+5,20));

hcar = phased.RadarTarget('MeanRCS',car_rcs,'PropagationSpeed',c,...
    'OperatingFrequency',fc);
hcarplatform = phased.Platform('InitialPosition',...
    [hradarplatform.InitialPosition(1)+car_dist;0;0.5],...
    'Velocity',[car_speed;0;0]);

%% Interference Model

%%
% The propagation model is assumed to be free space.

hchannel = phased.FreeSpace('PropagationSpeed',c,...
    'OperatingFrequency',fc,'SampleRate',fs,'TwoWayPropagation',true);

%% Radar System Setup
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

    

%% Next, run the simulation loop. 
Nsweep = 2;
xr = complex(zeros(hwav.SampleRate*hwav.SweepTime,Nsweep));
radar_pos = zeros(Nsweep,3);
radar_vel = zeros(Nsweep,3);
tgt_pos = zeros(Nsweep,3);
tgt_vel = zeros(Nsweep, 3);

for m = 1:Nsweep
    [radar_pos(m,:),radar_vel(m,:)] = step(...
        hradarplatform,hwav.SweepTime);       % radar moves during sweep
    [tgt_pos(m,:),tgt_vel(m,:)] = step(hcarplatform,... 
        hwav.SweepTime);                      % car moves during sweep
    x = step(hwav);                           % generate the FMCW signal
    xt = step(htx,x);                         % transmit the signal
    xp = step(hchannel,xt,radar_pos(m,:)',tgt_pos(m,:)',...
        radar_vel(m,:)',tgt_vel(m,:)');                   % propagate the signal
    xrefl = step(hcar,xp);                       % reflect the signal
    xrx = step(hrx,xrefl);                        % receive the signal
    xd = dechirp(xrx,x);                       % dechirp the signal
    
    
    
    [pxx,f] = periodogram(x, hamming(length(x)), nextpow2(length(x)), fs);
    [pxxt,ft] = periodogram(xt, hamming(length(xt)), nextpow2(length(xt)), fs);    
    [pxxr,fr] = periodogram(xp, hamming(length(xp)), nextpow2(length(xp)), fs);    
    [pxxrefl,frefl] = periodogram(xrefl, hamming(length(xrefl)), nextpow2(length(xrefl)), fs);    
    [pxxrx,frx] = periodogram(xrx, hamming(length(xrx)), nextpow2(length(xrx)), fs);
    [pxxd,fd] = periodogram(xd, hamming(length(xd)), nextpow2(length(xd)), fs);
    
    
    
    xr(:,m) = xd;                             % buffer the dechirped signal
end

%% Splitting up the sweeps;
xr_upsweep = xr(:,1:2:end);
xr_downsweep = xr(:,2:2:end);

%% Plotting Power
figure
plot(f, 10*log10(pxx), ft, 10*log10(pxxt), ...
    fr, 10*log10(pxxr), ...
    frefl, 10*log10(pxxrefl),...
    frx, 10*log10(pxxrx));
title('Periodogram Power Spectral Density Estimate')
legend('Generated Signal', 'Transmitted Signal', 'Propagated Signal',...
    'Reflected Signal', 'Received Signal');

%% Plotting Target Positions
figure
radar_pos_x = radar_pos(:,1);
radar_pos_y = radar_pos(:,2);
radar_pos_z = radar_pos(:,3);
tgt_pos_x = tgt_pos(:,1);
tgt_pos_y = tgt_pos(:,2);
tgt_pos_z = tgt_pos(:,3);
hold on
plot(radar_pos_x,radar_pos_z, 'b-');
plot(radar_pos_x(1),radar_pos_z(1), 'bo');
plot(radar_pos_x(Nsweep),radar_pos_z(Nsweep), 'bx');

plot(tgt_pos_x, tgt_pos_z, 'g-');
plot(tgt_pos_x(1), tgt_pos_z(1), 'go');
plot(tgt_pos_x(Nsweep), tgt_pos_z(Nsweep), 'gx');
xlabel('X (m)')
ylabel('Z (m)')
legend('Our Radar','Our Start', 'Our End','Target System', ...
    'Target Start', 'Target End')
grid
hold off



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




%%
% The up sweep and down sweep are processed separately to obtain the beat
% frequencies corresponding to both up and down sweep.
fbu_rng = rootmusic(pulsint(xr_upsweep,'coherent'),1,fs);
fbd_rng = rootmusic(pulsint(xr_downsweep,'coherent'),1,fs);
output.rng_est = beat2range([fbu_rng fbd_rng],sweep_slope,c);
fd = -(fbu_rng+fbd_rng)/2;
output.v_est = dop2speed(fd,lambda)/2;

disp(output)


%% Two-ray Propagation
% chant = phased.TwoRayChannel('PropagationSpeed',c,...
%     'OperatingFrequency',fc,'SampleRate',fs);
% chanr = phased.TwoRayChannel('PropagationSpeed',c,...
%     'OperatingFrequency',fc,'SampleRate',fs);
% Nsweep = 64;
% xr = helperFMCWTwoRaySimulate(Nsweep,hwav,hradarplatform,hcarplatform,...
%     htx,chant,chanr,hcar,hrx);
% plotResponse(hrdresp,xr);                     % Plot range Doppler map
% axis([-v_max v_max 0 range_max]);
% caxis(clim);



%% Accuracy
accuracy.true_v = radar_speed - car_speed;
accuracy.true_rng = car_dist;

accuracy.percent_error_v = ((accuracy.true_v - output.v_est)/accuracy.true_v)*100;
accuracy.percent_error_rng = ((car_dist - output.rng_est)/car_dist)*100;

disp(accuracy)
