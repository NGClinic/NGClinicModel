
%%-------------------------------------------------------------------------
% Chirp Generator
% mngai@hmc.edu 
% 
% I couln't use any "real" values since it would freeze my computer.
% 
% You can update these parameters for your model.
%%-------------------------------------------------------------------------
clear
clc
close all
freq_factor = 1;
df = 300;                                 % Bandwidth = 380 Mhz;
fc = 440; %1800*(10^6)*freq_factor;       % Center Frequency = 1.8 Ghz
fmin = fc-df/2;
fmax = fc+df/2;
fs = fmax*20;                             % Sampling Frequency = 2.2 Msps
dt = 1/fs;                                % Time Step
time_factor = 1;
Tm = 3*time_factor;                       % Modulation Index = 1.25 ms
t = 0:dt:2*Tm;

ramp_gen = sawtooth(2*pi*t/(2*Tm),0.5);
tx_signal = vco(ramp_gen,[fmin fmax],fs);

spectrogram(tx_signal,256*2,220*2,512*8,fs,'yaxis')
xlabel('Time(s)')
ylabel('Frequency (Hz)')
title('Spectrogram of Generated Chirp')
%axis([0 Tm*2 fmin*.6 fmax*1.4])
%sound(real(tx_signal)*10, fs)


%% Object Processing
c = 3*10^8;             % speed of light
distance = 4*10^4;         % distance of object
tdelay = 2*distance/c;

% Let's just make some arbitrary delays!
tshift = 3.1 %(tdelay/dt)*time_factor

%% Doppler Effect
%http://www.radartutorial.eu/11.coherent/co06.en.html
velocity = 10;              % Velocity of object
theta = pi/2;               % Angle of arival
vr = velocity*sin(theta);   % radial velocity
freq = fc;            % carrier freq
lambda = c/freq;

% Arbitrary
fdoppler = 10; %((2*vr)/lambda)*freq_factor;


%% Apply Freq and Time Shift

% % Determine the necessary resolution for the fft shift
% fftres = fdoppler;
% fftsize = 2^nextpow2(fs/fftres);
% fftres = fs/fftsize;


% Apply freq shift
fftsize = 2^nextpow2(length(tx_signal));
fftres = fs/fftsize;
fftshift = round(fdoppler/fftres);
fft_tx = fft(tx_signal,fftsize);
figure
plot(0:fs/fftsize:fs-fs/fftsize, abs(fft_tx));
%axis([0 fs/15 0 2000])
    
% Shift the FFT data
fft_tx_shift = circshift(fft_tx(1:fftsize/2), [0, fftshift]);

%Plot shifted FFT
%figure
%plot(0:fs/fftsize:(fs/2)-fs/fftsize, abs(fft_tx_shift));


%% Plot Doppler Shifted Data
tx_freqshifted = ifft(fft_tx_shift, fftsize);
tx_freqshifted = tx_freqshifted(1:length(tx_signal));
spectrogram(tx_freqshifted,256*2,220*2,512*8,fs,'yaxis')
xlabel('Time(s)')
ylabel('Frequency (Hz)')
title('Spectrogram of Doppler Shifted Chirp')
axis([0 Tm*2 fmin*.6 fmax*1.4])
%plot(1:length(tx_freqshifted), tx_freqshifted)

%% Apply time shift
reflected_signal = circshift(tx_freqshifted, [0, round(tshift/dt)]);
figure
spectrogram(reflected_signal,256*2,220*2,512*8,fs,'yaxis')
xlabel('Time(s)')
ylabel('Frequency (Hz)')
title('Spectrogram of Reflected Chirp')
axis([0 Tm*2 fmin*.6 fmax*1.4])

%% Plays the transmited signal and then the reflected signal
sound(real([tx_signal reflected_signal])*10, fs)


%%
windowSize = 512*4;
tStart = 4; %seconds
tWindow = tStart/dt:tStart/dt + windowSize;
tx_small = tx_signal(tWindow);
refl_small = reflected_signal(tWindow);
figure
plot(real(tx_small));
hold on
plot(real(refl_small), 'r');
hold on 
plot(imag(refl_small), 'g');
figure
fft_tx = fft(tx_small, 2^nextpow2(length(tx_small)));
plot(1:length(fft_tx), abs(fft_tx));
title('TX small fft at 4 sec')

figure
fft_refl = fft(refl_small, 2^nextpow2(length(refl_small)));
plot(1:length(fft_refl), abs(fft_refl));
title('Refl small fft 4 sec')






