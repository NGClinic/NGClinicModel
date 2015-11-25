%% FMCW Example
% Based on Automotive Radar Example from Matlab
%   Copyright 2012-2015 The MathWorks, Inc.
close all
clc
clear

FMCWsimsetup
%% Simulation Loop 
Nsweep = 64;

%% Initializing zero-vectors
radar_pos = zeros(Nsweep,3);
radar_vel = zeros(Nsweep,3);
tgt_pos = zeros(Nsweep,3);
tgt_vel = zeros(Nsweep, 3);
itfer_pos = zeros(Nsweep, 3);
itfer_vel = zeros(Nsweep,3);
xr = zeros(length(step(hwav)), Nsweep);
maxdist = 100;

% Simulation for multiple Sweeps
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


plotPower; % Plotting Spectral Density
plotVehiclePositions; % Plotting Vehicle Positions

%% Calculate Range and Doppler
% TODO Fix

xr_upsweep = xr(1:hwav.SweepTime*fs,:);
xr_downsweep = xr((hwav.SweepTime*fs):end, :);
if (1)
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

plotAccuracy;   % Plot accuracy of calculations
