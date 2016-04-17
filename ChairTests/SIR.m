% Cross-correlation of different tests

clear all
close all

% read the raw data .wave file here
% Scenario3 - OFF, Scenario3 - ON, Scenario2 - ON
% [Y3off,FS] = audioread('scen3_15m_off_2.wav');
% [Y3on,FS] = audioread('scen3_wave1_15m_on_2.wav');
% [Y2on,FS] = audioread('scen2_wave1_15m_2.wav');

[Y3off,FS] = audioread('Scenario3/scen3_15m_off_3.wav');
[Y3on,FS] = audioread('Scenario3/scen3_wave1_15m_on_3.wav');
[Y2on,FS] = audioread('Scenario2/scen2_wave1_15m_3.wav');

% Compare same test cases with these
% [Y2on,FS] = audioread('/Users/hmcloaner/Documents/ChairTests/Scenario3/scen3_wave1_15m_on_3.wav');
% [Y3on,FS] = audioread('/Users/hmcloaner/Documents/ChairTests/Scenario4/scen4_off_5.wav');
% [Y3off,FS] = audioread('/Users/hmcloaner/Documents/ChairTests/Scenario4/scen4_off_2.wav');

%constants
c = 3E8; %(ms) speed of light
%radar parameters
Tp = 20E-3; %(s) pulse time
N = Tp*FS; %# of samples per pulse
fstart = 2260E6; %(Hz) LFM start frequency for example
fstop = 2590E6; %Hz) LFM stop frequency for example
%fstart = 2402E6; %(Hz) LFM start frequency for ISM band
%fstop = 2495E6; %(Hz) LFM stop frequency for ISM band
BW = fstop-fstart; %(Hz) transmit bandwidth
f = linspace(fstart, fstop, N/2); %instantaneous transmit frequency

%range resolution
rr = c/(2*BW);
max_range = rr*N/2;

%the input appears to be inverted
trig3off = -1*Y3off(:,1);
s_3off = -1*Y3off(:,2);

trig3on = -1*Y3on(:,1);
s_3on = -1*Y3on(:,2);

trig2on = -1*Y2on(:,1);
s_2on = -1*Y2on(:,2);


%parse the data here by triggering off rising edge of sync pulse
count = 0;
thresh = 0;
start_3off = (trig3off > thresh); % Same size, either 1 (true) or 0 (false)

for ii = 100:(size(start_3off,1)-N)
    if start_3off(ii) == 1 && mean(start_3off(ii-11:ii-1)) == 0
        start2_3off(ii:ii+N-1) = s_3off(ii:ii+N-1);
        count = count + 1;
        sif_3off(count,:) = s_3off(ii:ii+N-1);
        % each row of sif is N data (one pulse)
        time_3off(count) = ii*1/FS;
        % corresponding vector of time 
    end
end

count = 0;
thresh = 0;
start_3on = (trig3on > thresh); % Same size, either 1 (true) or 0 (false)

for ii = 100:(size(start_3on,1)-N)
    if start_3on(ii) == 1 && mean(start_3on(ii-11:ii-1)) == 0
        start2_3on(ii:ii+N-1) = s_3on(ii:ii+N-1);
        count = count + 1;
        sif_3on(count,:) = s_3on(ii:ii+N-1);
        % each row of sif is N data (one pulse)
        time_3on(count) = ii*1/FS;
        % corresponding vector of time 
    end
end

count = 0;
thresh = 0;
start_2on = (trig2on > thresh); % Same size, either 1 (true) or 0 (false)

for ii = 100:(size(start_2on,1)-N)
    if start_2on(ii) == 1 && mean(start_2on(ii-11:ii-1)) == 0
        start2_2on(ii:ii+N-1) = s_2on(ii:ii+N-1);
        count = count + 1;
        sif_2on(count,:) = s_2on(ii:ii+N-1);
        % each row of sif is N data (one pulse)
        time_2on(count) = ii*1/FS;
        % corresponding vector of time 
    end
end

% %%%%%%%%%%%%%%%%%%%%%%%
% % plot power vs. time %
% %%%%%%%%%%%%%%%%%%%%%%%
% 
% for i = 1:size(sif_3off,1)
%     power_3off(i) = rms(sif_3off(i,:))^2;
% end
% for i = 1:size(sif_3on,1)
%     power_3on(i) = rms(sif_3on(i,:))^2;
% end
% for i = 1:size(sif_2on,1)
%     power_2on(i) = rms(sif_2on(i,:))^2;
% end
% 
% % Plot Beat Signal Power Amplitude vs. Time
% figure(1)
% subplot(3,1,1)
% plot(time_2on,power_2on)
% xlabel('Time (s)');
% ylabel('Amp');
% title('Scenario 2 - Interferer ON');
% xlim([0 25]);
% grid on
% subplot(3,1,2)
% plot(time_3on,power_3on)
% xlabel('Time (s)');
% ylabel('Amp');
% title('Scenario 3 - Interferer ON');
% xlim([0 25]);
% grid on
% subplot(3,1,3)
% plot(time_3off,power_3off)
% xlabel('Time (s)');
% ylabel('Amp');
% title('Scenario 3 - Interferer OFF');
% xlim([0 25]);
% grid on
% 
% % Plot Beat Signal Power (dB) vs. Time
% figure(2)
% subplot(3,1,1)
% plot(time_2on,dbv(power_2on))
% xlabel('Time (s)');
% ylabel('Power (dB)');
% title('Scenario 2 - Interferer ON');
% ylim([-18 -17]);
% xlim([0 25]);
% grid on
% subplot(3,1,2)
% plot(time_3on,dbv(power_3on))
% xlabel('Time (s)');
% ylabel('Power (dB)');
% title('Scenario 3 - Interferer ON');
% ylim([-18 -17]);
% xlim([0 25]);
% grid on
% subplot(3,1,3)
% plot(time_3off,dbv(power_3off))
% xlabel('Time (s)');
% ylabel('Power (dB)');
% title('Scenario 3 - Interferer OFF');
% ylim([-18 -17]);
% xlim([0 25]);
% grid on

%%%%%%%%%%%%%%
% SIR values %
%%%%%%%%%%%%%%

% Total Signal Power in Single Value
P2on = rms(reshape(sif_2on,1,[]))^2;
P3on = rms(reshape(sif_3on,1,[]))^2;
P3off = rms(reshape(sif_3off,1,[]))^2;

disp(['(2 ON)  I+N+M = ',num2str(dbv(P2on)), ' dB']);
disp(['(3 ON)  S+I+N+M = ',num2str(dbv(P3on)), ' dB']);
disp(['(3 OFF) S+N+M = ',num2str(dbv(P3off)),  'dB']);

sir = (P3on - P2on) / (P3on - P3off);
if sir <= 0
    disp(['SIR = ',num2str(sir)]);
else
    disp(['SIR = ', num2str(dbv(sir)), ' dB']);
end

% % Cross Correlation %
% % Align two tests to same length %
% [acor,lag] = xcorr(power_3off,power_3on);
% [~,I] = max(abs(acor));
% lagDiff = lag(I);
% timeDiff = lagDiff/FS;
% 
% % figure(3)
% % plot(lag,acor)
% % a3 = gca;
% % a3.XTick = sort([-3000:1000:3000 lagDiff]);
% 
% if length(power_3off) > length(power_3on)
%     power_3off_a = power_3off(lagDiff+1:end);
%     time_3off_a = time_3off(lagDiff+1:end);
%     figure(4)
%     subplot(2,1,1)
%     plot(time_3on,dbv(power_3on))
%     xlim([0 25]);
%     subplot(2,1,2)
%     plot(time_3off_a,dbv(power_3off_a))
%     xlim([0 25]);
% end
% if length(power_3off) < length(power_3on)
%     power_3on_a = power_3on(-lagDiff+1:end);
%     time_3on_a = time_3on(-lagDiff+1:end);  
%     figure(4)
%     subplot(2,1,1)
%     plot(time_3on_a,dbv(power_3on_a))
%     xlim([0 25]);
%     subplot(2,1,2)
%     plot(time_3off,dbv(power_3off))
%     xlim([0 25]);
% end
