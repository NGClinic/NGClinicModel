%MIT IAP Radar Course 2011
%Resource: Build a Small Radar System Capable of Sensing Range, Doppler, 
%and Synthetic Aperture Radar Imaging 
%
%Gregory L. Charvat

%Process Range vs. Time Intensity (RTI) plot

%NOTE: set up-ramp sweep from 2-3.2V to stay within ISM band
%change fstart and fstop bellow when in ISM band

clear all;
close all;
%read the raw data .wave file here
%*******change argument to directory where the file is***********
[Y,FS] = audioread('Scenario1\scen1_cont_1.wav');
%[Y,FS] = audioread('running_outside_20ms.wav');


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

%%
figure(100)
L = 2^nextpow2(length(s));
FT = fft(s,L);
w = linspace(0,FS/2,L/2);
plot(w,abs(FT(1:L/2)/L));
xlabel('freq (Hz)');
title('fft of beat signal');
xlim([0 500]);

% figure(101)
% LL = 2^nextpow2(length(trig));
% FT = fft(trig,LL);
% w = linspace(0,FS/2,LL/2);
% plot(w,abs(FT(1:LL/2)/LL));
% xlabel('freq (Hz)');
% title('fft of sync pulse');
% xlim([0 500]);

%%



%parse the data here by triggering off rising edge of sync pulse
count = 0;
thresh = 0;
start = (trig > thresh); % Same size, either 1 (true) or 0 (false)

for ii = 100:(size(start,1)-N)
    if start(ii) == 1 && mean(start(ii-11:ii-1)) == 0
        start2(ii:ii+N-1) = s(ii:ii+N-1);
        count = count + 1;
        sif(count,:) = s(ii:ii+N-1);
        % each row of sif is N data (one pulse)
        time(count) = ii*1/FS;
        % corresponding vector of time 
    end
end

% plot power vs. time
nf = 1/max(s);
sif_norm = nf*sif; % normalize first
for i = 1:size(sif_norm,1)
    power(i) = rms(sif_norm(i,:))^2;
end

figure(300)
plot(time(1:length(power)),dbv(power));
xlabel('time(s)');
ylabel('power(dB)');

% figure(11);     
% plot(sif(100,:));
% title('parsed data');
figure(12);
plot(s);
title('raw beat signal')
ylim([-0.4 0.4]);

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
imagesc(R,time,datamat,[-45, 0]);
colorbar;
colormap jet
ylabel('time (s)');
xlabel('range (m)');
xlim([0 50]);
title('RTI with 2-pulse cancelor clutter rejection');

% [M,I] = max(datamat,[],2);
% for i = 1:length(I)
%     maxrange(i) = R(I(i));
% end
% figure(200)
% plot(time(1:length(maxrange)),maxrange);

% %2 pulse mag only cancelor
% figure(30);
% clear v;
% for ii = 1:size(sif,1)-1
%     v1 = abs(ifft(sif(ii,:),zpad));
%     v2 = abs(ifft(sif(ii+1,:),zpad));
%     v(ii,:) = v2-v1;
% end
% S=v;
% R = linspace(0,max_range,zpad);
% for ii = 1:size(S,1)
%     S(ii,:) = S(ii,:).*R.^(3/2); %Optional: magnitude scale to range
% end
% S = dbv(S(:,1:size(v,2)/2));
% m = max(max(S));
% imagesc(R,time,S-m,[-20, 0]);
% colorbar;
% xlim([0 80]);
% ylabel('time (s)');
% xlabel('range (m)');
% title('RTI with 2-pulse mag only cancelor clutter rejection');

% % Animation %
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


%% flip %
imagesc(time,R,(S-m)',[-40, 0]);
colorbar;
colormap jet
ylabel('range (m)');
xlabel('time (s)');
set(gca,'YDir','normal');
ylim([0 100]);
title('RTI with 2-pulse cancelor clutter rejection');



