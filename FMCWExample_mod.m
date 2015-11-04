%% Automotive Adaptive Cruise Control Using FMCW Technology
% This example shows how to model an automotive adaptive cruise control
% system using the frequency modulated continuous wave (FMCW) technique.
% This example performs range and Doppler estimation of a moving vehicle.
% Unlike pulsed radar systems that are commonly seen in the defense
% industry, automotive radar systems often adopt FMCW technology. Compared
% to pulsed radars, FMCW radars are smaller, use less power, and are much
% cheaper to manufacture. As a consequence, FMCW radars can only monitor a
% much smaller distance.

%   Copyright 2012-2015 The MathWorks, Inc.

%% FMCW Waveform 
% Consider an automotive long range radar (LRR) used for automatic cruise
% control (ACC). This kind of radar usually occupies the band around 77
% GHz, as indicated in [1]. The radar system constantly estimates the
% distance between the vehicle it is mounted on and the vehicle in front of
% it, and alerts the driver when the two become too close. The figure below
% shows a sketch of ACC.
%
% <<FMCWExample_acc.png>>
%
% A popular waveform used in ACC system is FMCW. The principle of range
% measurement using the FMCW technique can be illustrated using the
% following figure.
%
% <<FMCWExample_upsweep.png>>
%
% The received signal is a time-delayed copy of the transmitted signal
% where the delay, $\Delta t$, is related to the range. Because the signal
% is always sweeping through a frequency band, at any moment during the
% sweep, the frequency difference, $f_b$, is a constant between the
% transmitted signal and the received signal. $f_b$ is usually called the
% beat frequency. Because the sweep is linear, one can derive the time
% delay from the beat frequency and then translate the delay to the range.
%
% In an ACC setup, the maximum range the radar needs to monitor is around
% 200 m and the system needs to be able to distinguish two targets that are
% 1 meter apart. From these requirements, one can compute the waveform
% parameters.

fc = 77e9;
c = 3e8;
lambda = c/fc;

%%
% The sweep time can be computed based on the time needed for the signal to
% travel the unambiguous maximum range. In general, for an FMCW radar
% system, the sweep time should be at least 5 to 6 times the round trip
% time. This example uses a factor of 5.5.

range_max = 200;
tm = 5.5*range2time(range_max,c);

%%
% The sweep bandwidth can be determined according to the range resolution
% and the sweep slope is calculated using both sweep bandwidth and sweep
% time.

range_res = 1;
bw = range2bw(range_res,c);
sweep_slope = bw/tm;

%%
% Because an FMCW signal often occupies a huge bandwidth, setting the
% sample rate blindly to twice the bandwidth often stresses the capability
% of A/D converter hardware. To address this issue, one can often choose a
% lower sample rate. Two things can be considered here:
%
% # For a complex sampled signal, the sample rate can be set to the same as
% the bandwidth.
% # FMCW radars estimate the target range using the beat frequency embedded
% in the dechirped signal. The maximum beat frequency the radar needs to
% detect is the sum of the beat frequency corresponding to the maximum
% range and the maximum Doppler frequency. Hence, the sample rate only
% needs to be twice the maximum beat frequency.
%
% In this example, the beat frequency corresponding to the maximum range is
% given by

fr_max = range2beat(range_max,sweep_slope,c);

%%
% In addition, the maximum speed of a traveling car is about 230 km/h.
% Hence the maximum Doppler shift and the maximum beat frequency can be
% computed as

v_max = 230*1000/3600;
fd_max = speed2dop(2*v_max,lambda);

fb_max = fr_max+fd_max;


%%
% This example adopts a sample rate of the larger of twice the maximum beat
% frequency and the bandwidth.
fs = max(2*fb_max,bw);

%%
% The following table summarizes the radar parameters.
% 
%  System parameters            Value
%  ----------------------------------
%  Operating frequency (GHz)    77
%  Maximum target range (m)     200
%  Range resolution (m)         1
%  Maximum target speed (km/h)  230
%  Sweep time (ms)              2
%  Sweep bandwidth (MHz)        400
%  Maximum beat frequency (MHz) 27.30
%  Sample rate (MHz)            150


%%
% With all the information above, one can set up the FMCW waveform used
% in the radar system.
hwav = phased.FMCWWaveform('SweepTime',tm,'SweepBandwidth',bw,...
    'SampleRate',fs);


%% Target Model
% The target of an ACC radar is usually a car in front of it. This example
% assumes the target car is moving 50 m ahead of the car with the
% radar, at a speed of 96 km/h along the x-axis. 
%
% The radar cross section of a car, according to [1], can be computed based
% on the distance between the radar and the target car.

car_dist = 43;
car_speed = 96*1000/3600;
car_rcs = db2pow(min(10*log10(car_dist)+5,20));

hcar = phased.RadarTarget('MeanRCS',car_rcs,'PropagationSpeed',c,...
    'OperatingFrequency',fc);
hcarplatform = phased.Platform('InitialPosition',[car_dist;0;0.5],...
    'Velocity',[car_speed;0;0]);

%%
% The propagation model is assumed to be free space.

hchannel = phased.FreeSpace('PropagationSpeed',c,...
    'OperatingFrequency',fc,'SampleRate',fs,'TwoWayPropagation',true);

%% Radar System Setup
% The rest of the radar system includes the transmitter, the receiver, and
% the antenna. This example uses the parameters presented in [1]. Note that
% this example models only main components and omits the effect from other
% components, such as coupler and mixer. In addition, for the sake of
% simplicity, the antenna is assumed to be isotropic and the gain of the
% antenna is included in the transmitter and the receiver.
ant_aperture = 6.06e-4;                         % in square meter
ant_gain = aperture2gain(ant_aperture,lambda);  % in dB

tx_ppower = db2pow(5)*1e-3;                     % in watts
tx_gain = 9+ant_gain;                           % in dB

rx_gain = 15+ant_gain;                          % in dB
rx_nf = 4.5;                                    % in dB

htx = phased.Transmitter('PeakPower',tx_ppower,'Gain',tx_gain);
hrx = phased.ReceiverPreamp('Gain',rx_gain,'NoiseFigure',rx_nf,...
    'SampleRate',fs);

%%
% Automotive radars are generally mounted on a vehicles, so they are often
% in motion. This example assumes the radar is traveling at a speed of 100
% km/h along x-axis. So the target car is approaching the radar at a
% relative speed of 4 km/h.
radar_speed = 100*1000/3600;
hradarplatform = phased.Platform('InitialPosition',[0;0;0.5],...
    'Velocity',[radar_speed;0;0]);

%% Radar Signal Simulation
% As briefly mentioned in earlier sections, an FMCW radar measures the
% range by examining the beat frequency in the dechirped signal. To extract
% this frequency, a dechirp operation is performed by mixing the received
% signal with the transmitted signal. After the mixing, the dechirped
% signal contains only individual frequency components that correspond to
% the target range.
% 
% In addition, even though it is possible to extract the Doppler
% information from a single sweep, the Doppler shift is often extracted
% among several sweeps because within one pulse, the Doppler frequency is
% indistinguishable from the beat frequency. To measure the range and
% Doppler, an FMCW radar typically performs the following operations:
%
% # The waveform generator generates the FMCW signal.
% # The transmitter and the antenna amplify the signal and radiate the
% signal into space.
% # The signal propagates to the target, gets reflected by the target, and
% travels back to the radar.
% # The receiving antenna collects the signal.
% # The received signal is dechirped and saved in a buffer.
% # Once a certain number of sweeps fill the buffer, the Fourier transform
% is performed in both range and Doppler to extract the beat frequency as
% well as the Doppler shift. One can then estimate the range and speed of
% the target using these results. Range and Doppler can also be shown as an
% image and give an intuitive indication of where the target is in the
% range and speed domain.
%
% The next section simulates the process outlined above. A total of 64
% sweeps are simulated and a range Doppler response is generated at the
% end.
%
% During the simulation, a spectrum analyzer is used to show the spectrum
% of each received sweep as well as its dechirped counterpart.

hspec = dsp.SpectrumAnalyzer('SampleRate',fs,...
    'PlotAsTwoSidedSpectrum',true,...
    'Title','Spectrum for received and dechirped signal',...
    'ShowLegend',true);
 
hwavtr = clone(hwav);
release(hwavtr);
tm = 2e-3;
hwavtr.SweepTime = tm;
sweep_slope = bw/tm;

%% Triangular Sweep
% In a triangular sweep, there are one up sweep and one down sweep to form
% one period, as shown in the following figure.
%
% <<FMCWExample_trisweep.png>>
%
% The two sweeps have the same slope except different signs. From the
% figure, one can see that the presence of Doppler frequency, $f_d$,
% affects the beat frequencies ($f_{bu}$ and $f_{bd}$) differently in up
% and down sweeps. Hence by combining the beat frequencies from both up and
% down sweep, the coupling effect from the Doppler can be averaged out and
% the range estimate can be obtained without ambiguity.

%%
% First, set the waveform to use triangular sweep.
hwavtr.SweepDirection = 'Triangle';

%%
% Now simulate the signal return. Because of the longer sweep time,
% fewer sweeps (16 vs. 64) are collected before processing.

Nsweep = 16;
% xr = helperFMCWSimulate(Nsweep,hwavtr,hradarplatform,hcarplatform,...
%     htx,hchannel,hcar,hrx);
xr = complex(zeros(hwavtr.SampleRate*hwavtr.SweepTime,Nsweep));

for m = 1:Nsweep
    [radar_pos,radar_vel] = step(...
        hradarplatform,hwavtr.SweepTime);       % radar moves during sweep
    [tgt_pos,tgt_vel] = step(hcarplatform,... 
        hwavtr.SweepTime);                      % car moves during sweep
    x = step(hwavtr);                           % generate the FMCW signal
    xt = step(htx,x);                         % transmit the signal
    xt = step(hchannel,xt,radar_pos,tgt_pos,...
        radar_vel,tgt_vel);                   % propagate the signal
    xt = step(hcar,xt);                       % reflect the signal
    xt = step(hrx,xt);                        % receive the signal
    xd = dechirp(xt,x);                       % dechirp the signal
    
    xr(:,m) = xd;                             % buffer the dechirped signal
end

%%
% The up sweep and down sweep are processed separately to obtain the beat
% frequencies corresponding to both up and down sweep.
fbu_rng = rootmusic(pulsint(xr(:,1:2:end),'coherent'),1,fs);
fbd_rng = rootmusic(pulsint(xr(:,2:2:end),'coherent'),1,fs);

%%
% Using both up sweep and down sweep beat frequencies simultaneously, the
% correct range estimate is obtained.
rng_est = beat2range([fbu_rng fbd_rng],sweep_slope,c)

%%
% Moreover, the Doppler shift and the velocity can also be recovered in a
% similar fashion.

fd = -(fbu_rng+fbd_rng)/2;
v_est = dop2speed(fd,lambda)/2




