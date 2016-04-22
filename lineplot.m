function [rangeline,lowtol,hightol,time] = lineplot(filename,cuttime)

%MIT IAP Radar Course 2011
%Resource: Build a Small Radar System Capable of Sensing Range, Doppler, 
%and Synthetic Aperture Radar Imaging 
%
%Gregory L. Charvat

%Process Range vs. Time Intensity (RTI) plot

%NOTE: set up-ramp sweep from 2-3.2V to stay within ISM band
%change fstart and fstop bellow when in ISM band

% clear all;
% close all;
%read the raw data .wave file here
%*******change argument to directory where the file is***********
[Y,FS] = audioread(filename);
%[Y,FS] = audioread('Scenario3\scen3_wave1_30m_3.wav');
%[Y,FS] = audioread('running_outside_20ms.wav');
%[Y,FS] = audioread('2_25_2016_test1.wav.wav');

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
trig = -1*Y(:,1);
s = -1*Y(:,2);

% t = linspace(0,0.2,6100);
% figure(1);
% plot(t,trig(1:6100));
% title('Sync Output from Voltage Function Generator');
% xlabel('Time (s)')
% ylabel('Voltage (V)');

clear Y;

%% FFT
% figure(100)
% L = 2^nextpow2(length(s));
% FT = fft(s,L);
% w = linspace(0,FS/2,L/2);
% plot(w,abs(FT(1:L/2)/L));
% xlabel('freq (Hz)');
% title('fft of beat signal');
% xlim([0 500]);

% figure(101)
% LL = 2^nextpow2(length(trig));
% FT = fft(trig,LL);
% w = linspace(0,FS/2,LL/2);
% plot(w,abs(FT(1:LL/2)/LL));
% xlabel('freq (Hz)');
% title('fft of sync pulse');
% xlim([0 500]);

%%

% parse the data here by triggering off rising edge of sync pulse
% trig : sync pulse 
% s : beat signal
% N : number of data in each pulse
count = 0;
thresh = 0;
start = (trig > thresh); % Same size, either 1 (true) or 0 (false)

for ii = 100:(size(start,1)-N)
    % make sure the code runs only when ii is at the rising edge
    if start(ii) == 1 && mean(start(ii-11:ii-1)) == 0
        count = count + 1;       
        % each row of sif is N data at one pulse
        sif(count,:) = s(ii:ii+N-1);
        
        % corresponding vector of time 
        time(count) = ii*1/FS;
    end
end

% plot power vs. time
for i = 1:size(sif,1)
    power(i) = rms(sif(i,:))^2;
end

% % POWER
% figure(300)
% plot(time(1:length(power)),dbv(power));
% xlabel('time(s)');
% ylabel('power(dB)');
% title('Power (dB) vs. Time');
% grid on

% figure(11);     
% plot(sif(100,:));
% title('parsed data');
% figure(12);
% plot(s);
% title('raw beat signal')
% ylim([-0.4 0.4]);

%subtract the average
ave = mean(sif,1);
for ii = 1:size(sif,1);
    sif(ii,:) = sif(ii,:) - ave;
end

zpad = 8*N/2;

%RTI plot
% figure(10);
v = 20*log10(abs(ifft(sif,zpad,2)));
S = v(:,1:size(v,2)/2);
m = max(max(v));
% imagesc(linspace(0,max_range,zpad),time,S-m,[-40, 0]);
% colorbar;
% colormap jet
% ylabel('time (s)');
% xlabel('range (m)');
% title('RTI without clutter rejection');
% xlim([0 30]);

%2 pulse cancelor RTI plot
figure(20);
sif2 = sif(2:size(sif,1),:)-sif(1:size(sif,1)-1,:);
v = ifft(sif2,zpad,2);
S=v;
R = linspace(0,max_range,zpad);
% for ii = 1:size(S,1)
%     S(ii,:) = S(ii,:).*R.^(3/2); %Optional: magnitude scale to range
% end
S = 20*log10(abs(S(:,1:size(v,2)/2)));
m = max(max(S));
datamat = S-m;
% imagesc(R,time,datamat,[-45, 0]);
% colorbar;
% colormap jet
% ylabel('time (s)');
% xlabel('range (m)');
% xlim([0 100]);
% title('RTI with 2-pulse cancelor clutter rejection');

%%%%%%%%
% flip %
%%%%%%%%
offset = 2.1608; % R(40) = 2.2165
R = R(1:end-39);
datamat = datamat(:,40:end);
imagesc(time,R,datamat',[-40, 0]);
colorbar;
colormap jet
ylabel('range (m)');
xlabel('time (s)');
set(gca,'YDir','normal');
ylim([0 50]);
title('RTI with 2-pulse cancelor clutter rejection');

%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot Range-time Line Plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%

% flip upside down first
datamat_flip = flipud(datamat);

while max(datamat_flip(1,:)) < -5
    datamat_flip = datamat_flip(2:end, 2:end);
    time = time(2:end);
end

maxrange = zeros(1,length(time)-1);

% first distance
[M,I] = max(datamat_flip(1,:));
maxrange(1) = R(2*I+39);

% tolerance
tol = 3;

for ii = 2:length(time)-1

    [M,I] = max(datamat_flip(ii,:));
    
    % enforce continuity
    while R(2*I+39) > maxrange(ii-1) + 1 || R(2*I+39) < maxrange(ii-1) - 1
        datamat_flip(ii,I) = -1000; % penalty
        [M,I] = max(datamat_flip(ii,:));
    end
        
    maxrange(ii) = R(2*I+39);
    
    % lower / upper bound of tolerance
    tolmat_prev = datamat_flip(ii,1:I-1);
    tolmat_after = datamat_flip(ii,I+1:I+400);
    
    lowbound_logic = tolmat_prev < M - tol;
    highbound_logic = tolmat_after < M - tol;
    
    lowbound = tolmat_prev(lowbound_logic);
    highbound = tolmat_after(highbound_logic);    
    
    [M_low, I_low] = max(lowbound);
    [M_high, I_high] = max(highbound);
    
    if isempty(I_high)
        I_high = 0;
    end
    if isempty(I_low)
        I_low = 0;
    end
    
    lowtol(ii) = R(2*I_low+39);
    hightol(ii) = R(2*(I+I_high)+39); 
    
end

% flip back the arrays
rangeline = fliplr(maxrange);
lowtol = fliplr(lowtol(2:end));
hightol = fliplr(hightol(2:end));

figure(10101)
plot(time(2:end),rangeline);
xlabel('time (s)');
ylabel('distance (m)');
grid on
hold on
plot(time(2:end-1),lowtol);
hold on
plot(time(2:end-1),hightol);


%%%% Extract linear portion

t_increment = time(end)/length(time);
cut_index = ceil(cuttime/t_increment);
time = time(cut_index:end);
rangeline = rangeline(cut_index:end);
lowtol = lowtol(cut_index:end);
hightol = hightol(cut_index:end);

figure(10102)
plot(time(1:end-1),rangeline);
xlabel('time (s)');
ylabel('distance (m)');
grid on
hold on
plot(time(1:end-2),lowtol);
hold on
plot(time(1:end-2),hightol);


% %%%%%%%%%%%%%
% % Animation %
% %%%%%%%%%%%%%
% min_x = 0;
% max_x = 100;
% hold on
% 
% for k = 0:0.1:22
% 
%     x = min_x:max_x;
%     y = ones(1,size(x,2))*k;
% 
%     h = plot(x,y,'m','LineWidth',2.5);
%     pause(0.1);
%     delete(h);
% end

end
