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

clear Y;

%% parse the data here by triggering off rising edge of sync pulse
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

%% RTI plot without clutter rejection
figure
v = 20*log10(abs(ifft(sif,zpad,2)));
S = v(:,1:size(v,2)/2);
m = max(max(v));
R = linspace(0,max_range,zpad/2);
datamat = S-m;
imagesc(R,time,datamat,[-40, 0]);
colorbar;
colormap jet
ylabel('time (s)');
xlabel('range (m)');
title('RTI without clutter rejection');
xlim([0 30]);

%% Calculate the range
% Let's use without clutter rejction

% Zoom into part we care about
startR = 4.434;     % Start of range
endR = 18.87;       % End of range

% Calculate new range and datamat
ind = R> startR;
newR = R(ind);
newDM = datamat(:, (ind));
ind = newR < endR;
newR = newR(ind);
newDM = newDM(:, (ind));

% Plot new range and datamat to make sure looks good
figure
imagesc(newR, time, newDM, [-40,0]); colorbar; colormap jet;
ylabel('time (s)');
xlabel('range (m)');
title('RTI without clutter rejection');
xlim([startR endR]);

%% Calculate max value at each time
[M, I] = max(newDM, [], 2);
maxRange = zeros(1, length(I));
for i = length(I);
    maxRange(i) = newR(I(i));
end
figure; plot(time,maxRange);



% %% 2 pulse cancelor RTI plot
% figure
% sif2 = sif(2:size(sif,1),:)-sif(1:size(sif,1)-1,:);
% v = ifft(sif2,zpad,2);
% S=v;
% S = 20*log10(abs(S(:,1:size(v,2)/2)));
% m = max(max(S));
% datamat = S-m;
% imagesc(R,time,datamat,[-45, 0]);
% colorbar;
% colormap jet
% ylabel('time (s)');
% xlabel('range (m)');
% xlim([0 50]);
% title('RTI with 2-pulse cancelor clutter rejection');
% 
% 
% %% flip %
% imagesc(time,R,(S-m)',[-40, 0]);
% colorbar;
% colormap jet
% ylabel('range (m)');
% xlabel('time (s)');
% set(gca,'YDir','normal');
% ylim([0 100]);
% title('RTI with 2-pulse cancelor clutter rejection');
% 
% 
% 
