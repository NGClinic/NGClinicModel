% function [output, beatsignal, fs_bs, signalRMS2, interfererRMS2, SIRdB] = radarSim(fc, tm, tm_INT, rangeMax, bw,...
%     bw_INT, Nsweep, LPmixer,...
%     rad_pat, radarPos, itferPos, tgtPos, radarVel, itferVel, tgtVel,...
%     txPower, txLossFactor,rxNF,...
%     rxLossFactor,...
%     PLOT, MUTUAL_INTERFERENCE, TARGET,...
%     PHASE_SHIFT, SAVE, fileName, targetType, cal)
% Based on Automotive Radar Example from Matlab
%   Copyright 2012-2015 The MathWorks, Inc.
%
% HMC 2015-2016 Clinic Project
%   Modified by Amy Ngai
%   mngai@hmc.edu
%
% Function:   Simulates radar environment

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
[tgt_rng, ~] = rangeangle(tgtPos(1,:)', radarPos(1,:)');

%% FMCW Generation --------------------------------------------------------
numChirpsSweeps = 2;
hwav = phased.FMCWWaveform('SweepTime',tm/2,'SweepBandwidth',bw,...
    'SampleRate',fs, 'SweepDirection', 'Triangle', 'NumSweeps', numChirpsSweeps); %full triangle

if PLOT.CHIRP
    s = step(hwav);
    subplot(211); plot(0:1/fs:tm-1/fs,real(s));
    xlabel('Time (s)'); ylabel('Amplitude (v)');
    title('FMCW signal'); axis tight;
    subplot(212); spectrogram(s,32,16,32,fs,'yaxis');
    title('FMCW signal spectrogram');
end

%% Inteferer waveform
numInt = size(itferPos,3);
hwavItfer = cell(numInt,1);
tm_INT = (2.^nextpow2(tm_INT./tm)).*tm;
for i = 1:numInt
    hwavItfer{i} = phased.FMCWWaveform('SweepTime',tm_INT(i)/2,...
        'SweepBandwidth',bw_INT(i), 'SampleRate',fs, 'SweepDirection',...
        'Triangle', 'NumSweeps',numChirpsSweeps*(tm/tm_INT(i))); %full triangle
end
  
%% Target Model Parameters ------------------------------------------------
% Radar cross section for a human
if or(strcmp(targetType,'human'), strcmp(targetType, 'person'))
    rcs = db2pow(-10);  %dBsm
elseif strcmp(targetType,'truck')
    rcs = db2pow(min(20*log10(tgt_rng)+5,20));
elseif strcmp(targetType,'motorcycle')
    rcs = db2pow(7);
elseif strcmp(targetType,'car')
    rcs = db2pow(min(10*log10(tgt_rng)+5,20));
else % default car
    rcs = db2pow(min(10*log10(tgt_rng)+5,20));
end
hTgt = phased.RadarTarget('MeanRCS',rcs,'PropagationSpeed',c,...
    'OperatingFrequency',fc);

%% Free Space Channel Set Up ----------------------------------------------
hchannel_twoway = phased.FreeSpace('PropagationSpeed',c,...
    'OperatingFrequency',fc,'SampleRate',fs,'TwoWayPropagation',true);
hchannel_oneway = phased.FreeSpace('PropagationSpeed',c,...
    'OperatingFrequency',fc,'SampleRate',fs,'TwoWayPropagation',false);
% ricianChannel = comm.RicianChannel('SampleRate', fs,...
%     'RandomStream',        'mt19937ar with seed', ...
%     'Seed',                10);
% hchannel_oneway = phased.TwoRayChannel('PropagationSpeed',c,...
%     'OperatingFrequency',fc,'SampleRate',fs);
rChan = ricianchan;
rChan.KFactor = 3;
hChan = comm.AWGNChannel('NoiseMethod', 'Signal to noise ratio (SNR)');
hChan.SNR = 30;
%% Antenna Model Set Up ---------------------------------------------------
% % MIT Values
antGain =  interp1(rad_pat.az, rad_pat.azdB, 0);  % value from MIT Slide deck || 10*log10((pi*ant_dia/lambda)^2);
txGain = 13.4 + antGain;                          % in dB

rxGain = 13.4 +antGain;                           % in dB

% Transceiver transmit
htx = phased.Transmitter('PeakPower',txPower,...
    'Gain',txGain,...
    'LossFactor',txLossFactor);

% Transceiver receive
hrx = phased.ReceiverPreamp('Gain',rxGain,...
    'NoiseFigure',rxNF,...
    'LossFactor', rxLossFactor,...
    'SampleRate',fs);

% Interferer transmit
htx_INT = phased.Transmitter('PeakPower',txPower,...
    'Gain',txGain,...
    'LossFactor',txLossFactor);

% Save memory
clear antGain txPower tx Gain txLossFactor rxPower rxGain rxNF
clear rxLossFactor rxGain rxNF rxLossFactor
%% Simulation Loop Memory Allocation---------------------------------------

% Modify beat signal sampling frequency
n = 5^(-round(log10(LPmixer/fs)/log10(5)));
fs_bs = fs/n;

%Initializing zero-vectors
lenData = length(step(hwav));
signal.xitfer = zeros(lenData, numInt);
signal.xitferRX = zeros(lenData, numInt);
xr.NoINT = zeros(lenData/n, Nsweep);
xr.INT = zeros(lenData/n, Nsweep);
phaseOffset = round(rand(numInt,1).*(lenData/n)); %Add a phase offset
beatsignal.NoINT = zeros((tm/2)*fs_bs*numChirpsSweeps*Nsweep, 1);
beatsignal.INT = zeros((tm/2)*fs_bs*numChirpsSweeps*Nsweep, 1);
beatsignal.INTonly = zeros((tm/2)*fs_bs*numChirpsSweeps*Nsweep, 1);

% Calculate target angle
tgt_ang = atan2(tgtPos(:,2) - radarPos(:,2), tgtPos(:,1) - radarPos(:,1));
    
SIRdBCal = zeros(Nsweep,1);
SIRdB = zeros(Nsweep,1);
signalRMS2 = zeros(Nsweep,1);
interfererRMS2Cal = zeros(Nsweep,1);
interfererRMS2 = zeros(Nsweep,1);
itferGaindB = zeros(Nsweep,1);
disp('Time to complete setting up variables in radarSim...')
toc
%% Simulation Loop --------------------------------------------------------
disp('...')
disp('Beginning simulation loop...')
for m = 1:Nsweep      
    % Generate Our Signal
    signal.x = step(hwav);                      % generate the FMCW signal

    if TARGET               
        % Release so you can change object parameters
        release(htx); release(hrx);

        % Update the gain for transceiver to target based on angle
        htx.Gain = 16 + interp1(rad_pat.az, rad_pat.azdB, radtodeg(tgt_ang(m)));
        hrx.Gain = 13.4 + interp1(rad_pat.az, rad_pat.azdB, radtodeg(-tgt_ang(m)));
    
        % Transmit Signal
        signal.xt = step(htx, signal.x);            % transmit the signal

        % Propagate Signal
        signal.xp = step(hchannel_twoway,...
            signal.xt,...
            radarPos(m,:)',....
            tgtPos(m,:)',...
            radarVel(m,:)',...
            tgtVel(m,:)');    
        
        % Reflect signal off target
        signal.xtgt = step(hTgt,signal.xp);         
        
        % Receive reflected signal
        signal.xtgtRX = step(hrx,signal.xtgt); 
        
        % Beat signal without inteference
        beatSignalTgt = downsample(dechirp(signal.xtgtRX,signal.x),n);
        xr.NoINT(:,m) = beatSignalTgt;                                    % buffer the dechirped signal
        beatsignal.NoINT((((tm/2)*fs_bs*numChirpsSweeps)*(m-1)+1):((tm/2)*fs_bs*numChirpsSweeps*m)) = beatSignalTgt;
    end
    
    % If mutual interference
    if MUTUAL_INTERFERENCE
        
        
        
        % Iterate through number of interferers     
        for int = 1:numInt   
            
            % Calculate fudge factor
            int_rng = sqrt((itferPos(m,2,int) - radarPos(m,2)).^2 + (itferPos(m,1,int) - radarPos(m,1)).^2);
            fudgeFactordB = spline(calData.distance, calData.coeff, int_rng);
            fudgeFactor = 1;%10^(fudgeFactordB/10);
            % Calculate angle
            int_ang = atan2(itferPos(m,2,int) - radarPos(m,2), itferPos(m,1,int) - radarPos(m,1));
            
            % Release so you can change parameters
            release(hrx); release(htx_INT); 
            
            % Update gain based on angle
            hrx.Gain = interp1(rad_pat.az, rad_pat.azdB, radtodeg(-int_ang));
            htx_INT.Gain = 13.4 + interp1(rad_pat.az, rad_pat.azdB, radtodeg(-int_ang));
            itferGaindB(m) = htx_INT.Gain+hrx.Gain;
            % Generate interfer signal
            xitfer_gen = step(hwavItfer{int});         
            
            % Transmit interfer signal
            xitfer_t = step(htx_INT, xitfer_gen);      
            
            % Propagate through channel    
            signal.xitfer(:,int) = step(hchannel_oneway, xitfer_t, ...
                itferPos(m,:,int)', radarPos(m,:)',...
                itferVel(m,:,int)', radarVel(m,:)'); 
%             signal.xitfer(:,int) = filter(rChan,signal.xitfer(:,int));
%             signal.xitfer(:,int) = step(hChan, signal.xitfer(:,int));
            
            
            % Introduce phase shift to interferer signal
            if PHASE_SHIFT
                signal.xitfer(:, int) =...
                    [signal.xitfer(phaseOffset(int):end, int)
                    signal.xitfer(1:(phaseOffset(int)-1), int)];
            end
            
            % Receive interferer signal
            signal.xitferRX(:,int) = step(hrx, signal.xitfer(:,int));
            signal.xitferCal(:,int) = signal.xitferRX(:,int)/sqrt(fudgeFactor);
            
            
        end
        % Beat signal of interferer
        sumXitfer = sum(signal.xitferRX,2);
        beatItfer = downsample(dechirp(sumXitfer, signal.x),n);
        
        sumXitferCal = sum(signal.xitferCal,2);
        beatItferCal = downsample(dechirp(sumXitferCal, signal.x),n);
        
        
        % Sum of beat signal of interferer(s)
        xr.INTonly(:,m) = beatItferCal;                           
        beatsignal.INTonly((((tm/2)*fs_bs*numChirpsSweeps)*(m-1)+1):((tm/2)*fs_bs*numChirpsSweeps*m)) = beatItferCal;
        
   
        
        if TARGET
            % Calculate SIR if target
            signalRMS2(m) = rms(beatSignalTgt).^2;
            interfererRMS2Cal(m) = rms(beatItferCal).^2;
            interfererRMS2(m) = rms(beatItfer).^2;
            SIRdBCal(m) = 10*log10(signalRMS2(m)/interfererRMS2Cal(m));
            SIRdB(m) = 10*log10(signalRMS2(m)/interfererRMS2(m));
        
        
            % Beat signal of chirp and interferer signal
             inputSignal = signal.xtgtRX + sumXitferCal;
             beatSignalTgtItfer = downsample(dechirp(inputSignal, signal.x),n);
            xr.INT(:,m) = beatSignalTgtItfer;                            % buffer the dechirped signal
            beatsignal.INT((((tm/2)*fs_bs*numChirpsSweeps)*(m-1)+1):((tm/2)*fs_bs*numChirpsSweeps*m)) = beatSignalTgtItfer;
        end

    end
   
end
disp('Time to complete simulation loop...')
toc

%% Plot SIR
clear xd
if (TARGET && PLOT.SIR && MUTUAL_INTERFERENCE)
    figure
  %  intDist = sqrt((itferPos(:,2) - radarPos(:,2)).^2 + (itferPos(:,1) - radarPos(:,1)).^2);
    intDist = itferPos(:,1);
    time = (0:Nsweep-1)*hwav.SweepTime;
    line(time, SIRdBCal, 'Color', 'k', 'Marker', '.')
    ylabel('SIR (dB)')
    ax1 = gca; %current axis
    ax1.YColor = 'k';
    set(ax1, 'xtick', [])
    ax1_pos = ax1.Position; % position of first axes
    ax2 = axes('Position',ax1_pos,...
        'XAxisLocation','bottom',...
        'YAxisLocation','right',...
        'Color','none');
    line(time, intDist, 'Parent', ax2, 'Color', 'r', 'Marker', '.')
    ax2 = gca; %current axis
    ax2.XColor = 'k';
    ax2.YColor = 'r';
    %set(ax2,'ActivePositionProperty','outerposition');
    ylabel('Interferer Distance (m)')
    xlabel('Time (s)')
    title('SIR')
    grid on
       
    % Plot FPSL
    figure
    hold on
    intDist = sqrt((itferPos(:,2) - radarPos(:,2)).^2 + (itferPos(:,1) - radarPos(:,1)).^2);
    FPSL = (4.*intDist.*pi./lambda).^2;
    logy = 10*log10(FPSL) - itferGaindB;
    logy = logy - mean(logy);
    diff = SIRdBCal(4) - logy(4);
    plot(intDist, logy + diff, 'k.-')
    
    % Plot Calibrated SIR
    plot(intDist, SIRdBCal, 'g.-')
    title('SIR Comparison')
    ylabel('SIR (dB)')
    
    % Plot Calibrated SIR
    plot(intDist, SIRdB, 'b.-')
    title('SIR Comparison')
    ylabel('SIR (dB)')
    
    
    % Plot HW Data
    hold on
    load('SIRdata.mat')
    plot(5:5:30, SIRlog(:,1), 'r.-')
    legend FPSL 'SW SIR Calibrated' 'SW SIR Uncalibrated' 'HW SIM' 
    errorbar(5:5:30, SIRlog(:,1), SIR_stdDev(:,1) ,'r')
    xlabel('Interferer Distance (m)')
    grid on

    
end
%%

% Save memory!
% clear hcar hcarplatform hchannel_oneway hchannel_twoway hitfer
% clear hitferplatform hradarplatform hrx hspec htx htx_INT
% clear i int_ang int_rng xitfer_gen

%% Plot Beat Signal TD and FD ---------------------------------------------
plotBeatSignal(beatsignal, fs_bs, PLOT.BEATSIGNAL, MUTUAL_INTERFERENCE, TARGET);

%% Plotting Vehicle Positions ---------------------------------------------
plotVehiclePositions(radarPos, tgtPos, itferPos, ...
        PLOT.VEHICLES, MUTUAL_INTERFERENCE,TARGET);

%% Process beat signal for calculations----------------------------------
if TARGET
    xr_upsweep_NoINT = xr.NoINT(1:hwav.SweepTime*fs_bs,:);
    xr_downsweep_NoINT = xr.NoINT((hwav.SweepTime*fs_bs):end, :);
end

if MUTUAL_INTERFERENCE
    xr_upsweep_INT = xr.INT(1:hwav.SweepTime*fs_bs,:);
    xr_downsweep_INT = xr.INT((hwav.SweepTime*fs_bs):end, :);
end



%% Calculation Range Distance ---------------------------------------------
if (TARGET)
    output.rng_est_NoINT = zeros(Nsweep,1);
    output.rng_true = zeros(Nsweep,1);
    output.v_est_NoINT = zeros(Nsweep, 1);
    output.v_true = -(radarVel(1,1)-tgtVel(1,1))*ones(Nsweep,1);

    % Calculation if no interference
    for i = 1:Nsweep
        fbu_rng_NoINT = rootmusic(pulsint(xr_upsweep_NoINT(:,i),'coherent'),1,fs_bs);
        fbd_rng_NoINT = rootmusic(pulsint(xr_downsweep_NoINT(:,i),'coherent'),1,fs_bs);
        output.rng_est_NoINT(i) = beat2range([fbu_rng_NoINT fbd_rng_NoINT],sweep_slope,c)/2;
        fd_NoINT = -(fbu_rng_NoINT+fbd_rng_NoINT)/2;
        output.v_est_NoINT(i) = dop2speed(fd_NoINT,lambda)/2;
        output.rng_true(i) = sqrt(sum((radarPos(i,:)-tgtPos(i,:)).^2));
    end

% Calculation with interference
    if (MUTUAL_INTERFERENCE)
        output.rng_est_INT = zeros(Nsweep,1);
        output.v_est_INT = zeros(Nsweep, 1);
        for i = 1:Nsweep
            fbu_rng_INT = rootmusic(pulsint(xr_upsweep_INT(:,i),'coherent'),1,fs_bs);
            fbd_rng_INT = rootmusic(pulsint(xr_downsweep_INT(:,i),'coherent'),1,fs_bs);
            output.rng_est_INT(i) = beat2range([fbu_rng_INT fbd_rng_INT],sweep_slope,c)/2;
            fd_INT = -(fbu_rng_INT+fbd_rng_INT)/2;
            output.v_est_INT(i) = dop2speed(fd_INT,lambda)/2;
        end
    end
else
    output = 0;
end
%% Range Doppler Estimation
% hrdresp = phased.RangeDopplerResponse('PropagationSpeed',c,...
%     'DopplerOutput','Speed','OperatingFrequency',fc,'SampleRate',fs_bs,...
%     'RangeMethod','FFT','SweepSlope',sweep_slope,...
%     'RangeFFTLengthSource','Property','RangeFFTLength',2048,...
%     'DopplerFFTLengthSource','Property','DopplerFFTLength',256);
% figure
% plotResponse(hrdresp, xr.NoINT);
% clim = caxis;

clear fd_INT fbd_rng_INT fbu_rng_INT fbu_rng_NoINT fbd_rng_NoINT fd_NoINT
clear xr_upsweep_NoINT xr_downsweep_NoINT xr_upsweep_INT xr_downsweep_INT
%% Plot accuracy of calculations ------------------------------------------
if and(PLOT.ACCURACY, TARGET)
    figure
    subplot(211);
    grid on
    hold on
    suptitle(['Accuracy with ' num2str(Nsweep) ' Sweeps'])
    t = (0:Nsweep-1)*hwav.SweepTime;
    plot(t, output.rng_true, '.-', 'DisplayName', 'Target Range (m)')
    plot(t, output.rng_est_NoINT,'.-', 'DisplayName', 'Calc w/o Int (m)'); 
    if MUTUAL_INTERFERENCE
        plot(t, output.rng_est_INT, '.-', 'DisplayName', 'Calc w/ Int (m) ')
    end
    legend('Location', 'eastoutside'); 
    hold off
    xlim([0, t(end)])
    title('Range'); ylabel('m'); xlabel('s');
   
    subplot(212);
    grid on
    hold on
    plot(t, output.v_true, ...
        '.-', 'DisplayName', 'Target Speed (m/s)')
    plot(t, -output.v_est_NoINT, '.-', 'DisplayName', 'Calc w/o Int (m/s)');
    if MUTUAL_INTERFERENCE
        plot(t, -output.v_est_INT, '.-', 'DisplayName', 'Calc w/ Int (m/s)');
    end
    hold off
    legend('Location', 'eastoutside'); 
    xlim([0, t(end)])
    title('Velocity'); ylabel('m/s');  xlabel('s');
    
     
end


%% Save to file
if SAVE
    save(fileName)
end
disp('Time to complete radarSim fuction call....')
toc
