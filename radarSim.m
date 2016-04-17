function [output, beatsignal, fs_bs] = radarSim(fc, tm, tm_INT, rangeMax, bw,...
    bw_INT, Nsweep, LPmixer,...
    rad_pat, radar_speed, radar_init_pos, car_speed, car_init_pos,...
    itfer_speed, itfer_init_pos, txPower, txLossFactor,rxNF,...
    rxLossFactor,...
    PLOT, MUTUAL_INTERFERENCE, ...
    PHASE_SHIFT, SAVE, fileName, target)

%% FMCW Example -----------------------------------------------------------
% Based on Automotive Radar Example from Matlab
%   Copyright 2012-2015 The MathWorks, Inc.
tic

%% System waveform parameters ---------------------------------------------
c = 3e8;   
lambda = c/fc; 
sweep_slope = bw/tm;        
fr_max = range2beat(rangeMax,sweep_slope,c); 
v_max = 230*1000/3600; 
fd_max = speed2dop(2*v_max,lambda);
fb_max = fr_max+fd_max;
fs = max(2*fb_max,bw);

clear range_max rangeRes fr_max v_max fd_max fb_max TPLink
%% Vehicle placement ------------------------------------------------------
[tgt_rng, ~] = rangeangle(car_init_pos, radar_init_pos);
[int_rng, ~] = rangeangle(itfer_init_pos, radar_init_pos);

%% FMCW Generation --------------------------------------------------------
hwav = phased.FMCWWaveform('SweepTime',tm/2,'SweepBandwidth',bw,...
    'SampleRate',fs, 'SweepDirection', 'Triangle', 'NumSweeps', 2); %full triangle
if PLOT.CHIRP
    s = step(hwav);
    subplot(211); plot(0:1/fs:tm-1/fs,real(s));
    xlabel('Time (s)'); ylabel('Amplitude (v)');
    title('FMCW signal'); axis tight;
    subplot(212); spectrogram(s,32,16,32,fs,'yaxis');
    title('FMCW signal spectrogram');
end


tempLen = length(step(hwav));
n = tempLen/round(tempLen/(fs/LPmixer));
fs_bs = fs/n;


%% >>>> Change this so it iterates and creates multiple for each set of parameters
hwav_INT = phased.FMCWWaveform('SweepTime',tm_INT/2,'SweepBandwidth',bw_INT,...
    'SampleRate',fs, 'SweepDirection', 'Triangle', 'NumSweeps',2*(tm/tm_INT)); %full triangle

%% Radar Parameters -------------------------------------------------------
hradarplatform = phased.Platform('InitialPosition',radar_init_pos,...
    'Velocity',[radar_speed;0;0]);

%% Target Model Parameters ------------------------------------------------
% Radar cross section for a human
if or(strcmp(target,'human'), strcmp(target, 'person'))
    rcs = db2pow(-10);  %dBsm
elseif strcmp(target,'truck')
    rcs = db2pow(min(20*log10(tgt_rng)+5,20));
elseif strcmp(target,'motorcycle')
    rcs = db2pow(7);
elseif strcmp(target,'car')
    rcs = db2pow(min(10*log10(tgt_rng)+5,20));
else
    rcs = db2pow(min(10*log10(tgt_rng)+5,20));
end
hcar = phased.RadarTarget('MeanRCS',rcs,'PropagationSpeed',c,...
    'OperatingFrequency',fc);
hcarplatform = phased.Platform('InitialPosition',...
    car_init_pos,...
    'Velocity',[car_speed;0;0]);

%% Interference Model -----------------------------------------------------
itfer_rcs = db2pow(min(10*log10(int_rng)+5,20));
hitfer = phased.RadarTarget('MeanRCS',itfer_rcs,'PropagationSpeed',c,...
    'OperatingFrequency',fc);
hitferplatform = phased.Platform('InitialPosition',...
    itfer_init_pos,...
    'Velocity',[itfer_speed;0;0]);


%% Free Space Channel Set Up ----------------------------------------------
hchannel_twoway = phased.FreeSpace('PropagationSpeed',c,...
    'OperatingFrequency',fc,'SampleRate',fs,'TwoWayPropagation',true);
hchannel_oneway = phased.FreeSpace('PropagationSpeed',c,...
    'OperatingFrequency',fc,'SampleRate',fs,'TwoWayPropagation',false);


%% Antenna Model Set Up ---------------------------------------------------
% % MIT Values
antGain =  interp1(rad_pat.az, rad_pat.azdB, 0);  % value from MIT Slide deck || 10*log10((pi*ant_dia/lambda)^2);
txGain = 13.4 + antGain;                           % in dB

rxGain = 1+antGain;                          % in dB

htx = phased.Transmitter('PeakPower',txPower,...
    'Gain',txGain,...
    'LossFactor',txLossFactor);
hrx = phased.ReceiverPreamp('Gain',rxGain,...
    'NoiseFigure',rxNF,...
    'LossFactor', rxLossFactor,...
    'SampleRate',fs);

htx_INT = phased.Transmitter('PeakPower',txPower,...
    'Gain',txGain,...
    'LossFactor',txLossFactor);

clear antGain txPower tx Gain txLossFactor rxPower rxGain rxNF
clear rxLossFactor rxGain rxNF rxLossFactor
%% Simulation Loop Memory Allocation---------------------------------------

%Initializing zero-vectors
radarPos = zeros(Nsweep,3);
radarVel = zeros(Nsweep,3);
tgtPos = zeros(Nsweep,3);
tgtVel = zeros(Nsweep, 3);
itferPos = zeros(Nsweep, 3);
itferVel = zeros(Nsweep,3);
xr.NoINT = zeros(length(step(hwav))/n, Nsweep);
xr.INT = zeros(length(step(hwav))/n, Nsweep);
beatsignal.NoINT = zeros((tm/2)*fs_bs*2*Nsweep, 1);
beatsignal.INT = zeros((tm/2)*fs_bs*2*Nsweep, 1);
beatsignal.INTonly = zeros((tm/2)*fs_bs*2*Nsweep, 1);
toc

%% Simulation Loop --------------------------------------------------------
for m = 1:Nsweep   
    % Move objects
    [radarPos(m,:),radarVel(m,:)] = step(...
        hradarplatform,hwav.SweepTime*hwav.NumSweeps);   % radar moves during sweep
    [tgtPos(m,:),tgtVel(m,:)] = step(hcarplatform,... 
        hwav.SweepTime*hwav.NumSweeps);                  % car moves during sweep
    [itferPos(m,:), itferVel(m,:)] = step(hitferplatform,...
        hwav.SweepTime*hwav.NumSweeps);                  % interferer moves during sweep
  
    % Calculate angle
    tgt_ang = atan2(tgtPos(m,2) - radarPos(m,2), tgtPos(m,1) - radarPos(m,1));
    int_ang = atan2(itferPos(m,2) - radarPos(m,2), itferPos(m,1) - radarPos(m,1));
    
    % Release so you can change object parameters
    release(htx); release(hrx); release(htx_INT);
    htx.Gain = 13.4 + interp1(rad_pat.az, rad_pat.azdB, radtodeg(tgt_ang(1)));
    hrx.Gain = 1 + interp1(rad_pat.az, rad_pat.azdB, radtodeg(-tgt_ang(1)));
    htx_INT.Gain = 13.4 + interp1(rad_pat.az, rad_pat.azdB, radtodeg(int_ang(1)))/2;
    
    % Generate Our Signal
    signal.x = step(hwav);                      % generate the FMCW signal
    signal.xt = step(htx,signal.x);             % transmit the signal
       
    signal.xp = step(hchannel_twoway,...
        signal.xt,...
        radarPos(m,:)',....
        tgtPos(m,:)',...
        radarVel(m,:)',...
        tgtVel(m,:)');                     % Propagate signal
    signal.xtgt = step(hcar,signal.xp);        % Reflect the signal

    % Beat signal without inteference
    signal.xrx = step(hrx,signal.xtgt); % receive the signal
    xd = downsample(dechirp(signal.xrx,signal.x),n); % dechirp the signal
        
    xr.NoINT(:,m) = xd;                             % buffer the dechirped signal
    beatsignal.NoINT((((tm/2)*fs_bs*2)*(m-1)+1):((tm/2)*fs_bs*2*m)) = xd;
      
    
    %%>>>>> CHANGE THIS 
    if MUTUAL_INTERFERENCE
        % Beat signal with interference
        %%>>>> Create each new interfering wave
        xitfer_gen = step(hwav_INT);                % Generate interfer signal
        %%%>>>> step through each on
        xitfer_t = step(htx_INT, xitfer_gen);       % Transmit interfer signal
        %%%>>>>>
        signal.xitfer = step(hchannel_oneway, xitfer_t, ...
            itferPos(m,:)', radarPos(m,:)',...
            itferVel(m,:)', radarVel(m,:)');  % Propagate through channel       
        
        if PHASE_SHIFT
            signal.xitfer = circshift(signal.xitfer,[length(signal.xitfer)/2 length(signal.xitfer)/2]);
        end
        %%%>>>>>>>>>>>>>> Receving interferer signal 'sum(interferers)'
        signal.xrx = step(hrx,(signal.xtgt + signal.xitfer));  % receive the signal
        xd = downsample(dechirp(signal.xrx,signal.x),n); % dechirp the signal
        
        
        xr.INT(:,m) = xd;                             % buffer the dechirped signal
        beatsignal.INT((((tm/2)*fs_bs*2)*(m-1)+1):((tm/2)*fs_bs*2*m)) = xd;
        
        % Isolate just the chirp
        % >>>>>>> update this
        signal.xrx = step(hrx,(signal.xitfer));  % receive the signal
        xd = downsample(dechirp(signal.xrx,signal.x),n); % dechirp the signal
                
        xr.INTonly(:,m) = xd;                             % buffer the dechirped signal
        beatsignal.INTonly((((tm/2)*fs_bs*2)*(m-1)+1):((tm/2)*fs_bs*2*m)) = xd;
    end
   
end
clear xd
toc

clear hcar hcarplatform hchannel_oneway hchannel_twoway hitfer
clear hitferplatform hradarplatform hrx hspec htx htx_INT
clear i int_ang int_rng xitfer_gen
%% Plotting Spectral Density ----------------------------------------------
if (PLOT.POWER)
    field = fieldnames(signal);
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

%% Plot Beat Signal TD and FD ---------------------------------------------
plotBeatSignal(beatsignal, fs_bs, PLOT.BEATSIGNAL, MUTUAL_INTERFERENCE)

%% Plotting Vehicle Positions ---------------------------------------------
plotVehiclePositions(radarPos, tgtPos, itferPos, ...
        PLOT.VEHICLES, MUTUAL_INTERFERENCE);


%% Process beat signal for calculations
xr_upsweep_NoINT = xr.NoINT(1:hwav.SweepTime*fs_bs,:);
xr_downsweep_NoINT = xr.NoINT((hwav.SweepTime*fs_bs):end, :);

if MUTUAL_INTERFERENCE
    xr_upsweep_INT = xr.INT(1:hwav.SweepTime*fs_bs,:);
    xr_downsweep_INT = xr.INT((hwav.SweepTime*fs_bs):end, :);
end

%% Calculation Range Distance ---------------------------------------------
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
    rng_true_NoINT(i) = sqrt(sum((radarPos(i,:)-tgtPos(i,:)).^2));
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

clear fd_INT fbd_rng_INT fbu_rng_INT fbu_rng_NoINT fbd_rng_NoINT fd_NoINT
clear xr_upsweep_NoINT xr_downsweep_NoINT xr_upsweep_INT xr_downsweep_INT
%% Plot accuracy of calculations ------------------------------------------
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

%% Save to file
if SAVE
    save(fileName)
end
toc
end