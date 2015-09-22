% Top Level File for the FMCW Radar
close all
clear
clc
c = 3*10^8;         % Speed of light (m/s)
df = 380;           % Bandwidth (Hz)
fc1 = 1000;      % Center frequency (Hz)
fc2 = 1300; 
fs = (fc1+df/2)*4; % Sampling rate (Hz)
Tm1 = (1/fc1)*10^4;   % Modulation index (s)
Tm2 = (1/fc2)*10^4;   % Modulation index (s)
theta = pi/2;       % Angle of Arrival for object (radians)
distance = 10;      % Distance of object (m)
velocity = 0;       % Velocity of object (m/s)
gamma1 = df/Tm1;      % Chirp Rate
gamma2 = df/Tm1;

tic
%% Object Processing

% Generate Chirp1
fmin = fc1-df/2;             % Hz
fmax = fc1+df/2;             % Hz
dt = 1/fs;                  % Time Step (s)
t = 0:dt:2*Tm1;              % Duration of chirp (s)

% Generate Chirp2
fmin2 = fc2-df/2;
fmax2 = fc2+df/2;
dt2 = 1/fs;
t2 = 0:dt2:2*Tm2;

% Generate signal 1
ramp_gen = sawtooth(2*pi*t/(2*Tm1),0.5);
tx_signal1 = vco(ramp_gen,[fmin fmax],fs);
len = length(tx_signal1);
fftsize = 2^nextpow2(len);
tx_signal1 = fft(tx_signal1, fftsize);
tx_signal1 = ifft(tx_signal1(1:fftsize/2), fftsize);
tx_signal1 = tx_signal1(1:len);

% Generate signal 2 
ramp_gen = sawtooth(2*pi*t/(2*Tm2),0.5);
tx_signal2 = vco(ramp_gen,[fmin2 fmax2],fs);
len2 = length(tx_signal2);
fftsize = 2^nextpow2(len2);
tx_signal2 = fft(tx_signal2, fftsize);
tx_signal2 = ifft(tx_signal2(1:fftsize/2), fftsize);
tx_signal2 = tx_signal2(1:len2);

tx_sum = tx_signal1 + 0.5*tx_signal2; 

figure
plot(1:length(tx_sum), real(tx_sum), 1:length(tx_sum), imag(tx_sum))
title('Plot TX Signal')

%% Plot Generated Chirp using axis parameters of Signal 1
figure
spectrogram(tx_signal1,256*4,220*4,512*16,fs,'yaxis')
xlabel('Time(s)')
ylabel('Frequency (Hz)')
title('Spectrogram of Signal 1')


%% Plot Generated Chirp using axis parameters of Signal 1
figure
spectrogram(tx_signal2,256*4,220*4,512*16,fs,'yaxis')
xlabel('Time(s)')
ylabel('Frequency (Hz)')
title('Spectrogram of Signal 2')




%% Plot Generated Chirp using axis parameters of Signal 1
figure
spectrogram(tx_sum,256*4,220*4,512*16,fs,'yaxis');
xlabel('Time(s)')
ylabel('Frequency (Hz)')
title('Spectrogram of Combined Signal')
toc

%% Get Data from Spectrogram
[tx_sum_spect, freqs_sum ,t] = spectrogram(tx_sum',256*4,220*4,512*16,fs,'yaxis');
%%
figure(5)
plot((1:length(freqs_sum))*fs/length(freqs_sum), abs(tx_sum_spect(:,100)), 'r')
hold on 
plot((1:length(freqs_sum))*fs/length(freqs_sum), abs(tx_sum_spect(:,200)), 'g')
title('Individual Slices of Spectrogram')
xlabel('Frequencies')
ylabel('Amplitudes')





