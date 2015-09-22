% Top Level File for the FMCW Radar
close all
clear
clc
c = 3*10^8;         % Speed of light (m/s)
<<<<<<< HEAD
df = 400*10^6;           % Bandwidth (Hz)
fc = 1.2*10^9;           % Center frequency (Hz)
fs = (fc+df/2)*20;  % Sampling rate (Hz)
lambda = c/fc;      % Wavelength (m)
Tm = (1/fc)*10^5;   % Modulation index (s)
theta = pi/2;       % Angle of Arrival for object (radians)
distance = 200;  % Distance of object (m)
velocity = 10^6;    % Velocity of object (m/s)
gamma = df/Tm;      % Chirp Rate

tic
=======
df = 3000;           % Bandwidth (Hz)
fc = 500*10^2;           % Center frequency (Hz)
fs = (fc+df/2)*20;  % Sampling rate (Hz)
lambda = c/fc;      % Wavelength (m)
Tm = (1/fc)*1000;   % Modulation index (s)
theta = pi/2;       % Angle of Arrival for object (radians)
distance = 400;  % Distance of object (m)
velocity = 0;    % Velocity of object (m/s)
gamma = df/Tm;      % Chirp Rate


>>>>>>> 51b62deef3abaeefd4e0c52b2f63d15a395d11c2
%% Object Processing

% Generate Chirp
fmin = fc-df/2;             % Hz
fmax = fc+df/2;             % Hz
dt = 1/fs;                  % Time Step (s)
t = 0:dt:2*Tm;              % Duration of chirp (s)

% Generate signal
ramp_gen = sawtooth(2*pi*t/(2*Tm),0.5);
tx_signal = vco(ramp_gen,[fmin fmax],fs);
len = length(tx_signal);
fftsize = 2^nextpow2(len);
tx_signal = fft(tx_signal, fftsize);
tx_signal = ifft(tx_signal(1:fftsize/2), fftsize);
tx_signal = tx_signal(1:len);


figure
plot(1:length(tx_signal), real(tx_signal), 1:length(tx_signal), imag(tx_signal))
title('Plot TX Signal')

%%
figure
spectrogram(tx_signal,256*4,220*4,512*16,fs,'yaxis')
xlabel('Time(s)')
ylabel('Frequency (Hz)')
title('Spectrogram of Generated Chirp')
axis([0 Tm*2 fmin*.8 fmax*1.2])



%% Calculate Shifts from Object Reflection
% Time Shift
c = 3*10^8;                 % speed of light (m/s)
tdelay = 2*distance/c;      % time delay (s)
tshift = tdelay/dt;

% Doppler Shift
vr = velocity*sin(theta);   % radial velocity (m/s)
lambda = c/fc;              % wavelength of carrier signal (m)
fdoppler = (2*vr)/lambda; % Doppler shift (Hz)


%% Apply Freq Shift

% Determine the necessary resolution for the fft shift
if (fdoppler ~= 0)
    fftsize = 2^nextpow2(max(length(tx_signal),fs/fdoppler));
else 
    fftsize = 2^nextpow2(length(tx_signal));
end

<<<<<<< HEAD

%%
=======
>>>>>>> 51b62deef3abaeefd4e0c52b2f63d15a395d11c2
fftres = fs/fftsize;
% fftsize = 2^nextpow2(length(tx_signal));
% fftres = fs/fftsize;

fftshift = round(fdoppler/fftres);
fdoppler = fftshift*fftres;
fft_tx = fft(tx_signal,fftsize);

% Shift the FFT data
fft_tx_shift = circshift(fft_tx(1:fftsize/2), [0, fftshift]);


%% Plot Doppler Shifted Data
tx_freqshifted = ifft(fft_tx_shift, fftsize);
tx_freqshifted = tx_freqshifted(1:length(tx_signal));
spectrogram(tx_freqshifted,256*2,220*2,512*8,fs,'yaxis')
xlabel('Time(s)')
ylabel('Frequency (Hz)')
title('Spectrogram of Doppler Shifted Chirp')
axis([0 Tm*2 fmin*.6 fmax*1.4])


%% Apply time shift
<<<<<<< HEAD
len = length(tx_freqshifted);
%reflected_signal = [zeros(round(tshift/dt),1)' tx_freqshifted(0:len-round(tshift/dt))]
reflected_signal = circshift(tx_freqshifted, [0, round(tshift)]);
=======
reflected_signal = circshift(tx_freqshifted, [0, -round(tshift/dt)]);
>>>>>>> 51b62deef3abaeefd4e0c52b2f63d15a395d11c2
figure
spectrogram(reflected_signal,256*2,220*2,512*8,fs,'yaxis')
xlabel('Time(s)')
ylabel('Frequency (Hz)')
title('Spectrogram of Reflected Chirp')
axis([0 Tm*2 fmin*.6 fmax*1.4])

<<<<<<< HEAD
toc 
tic
=======
>>>>>>> 51b62deef3abaeefd4e0c52b2f63d15a395d11c2

%% RX Object Processing
window = 256*2;
noverlap = 220*2;
spectfftsize = fftsize;
[chirpRefl,freqsRefl,t] = spectrogram(reflected_signal,window,noverlap,fftsize,fs);
[chirpIn,freqIn,tin] = spectrogram(tx_signal,window,noverlap,fftsize,fs);

txFreqs = [];
rxFreqs = [];

for n = 1:length(chirpIn(1,:))
    [pks,locs] = findpeaks(abs(chirpIn(:,n)));
    [maxPk,ind] = max(pks);
    txFreqs = [txFreqs,freqIn(locs(ind))];
end

for n = 1:length(chirpRefl(1,:))
    [pks,locs] = findpeaks(abs(chirpRefl(:,n)));
    [maxPk,ind] = max(pks);
    rxFreqs = [rxFreqs,freqsRefl(locs(ind))];
end

<<<<<<< HEAD
%% Plot Max TX/RX Frequency
figure(4)
plot((1:length(rxFreqs))*dt,rxFreqs, 'r')
xlabel('Time(s)')
ylabel('Frequency (Hz)')
hold on
plot((1:length(txFreqs))*dt,txFreqs, 'b')
xlabel('Time(s)')
ylabel('Frequency (Hz)')
=======
figure
plot(rxFreqs, 'r')
hold on
plot(txFreqs, 'b')
>>>>>>> 51b62deef3abaeefd4e0c52b2f63d15a395d11c2
legend('rxFreqs', 'txFreqs')
title('Compare Max Frequency')
hold off

<<<<<<< HEAD
%%
beat = abs(rxFreqs - txFreqs);
figure(5)
plot((1:length(beat))*dt, beat)
xlabel('Time(s)')
ylabel('Frequency (Hz)')
title('beat')

%% Detect plateaus
=======
beat = rxFreqs - txFreqs;

figure
plot(beat)
title('beat')

%%
>>>>>>> 51b62deef3abaeefd4e0c52b2f63d15a395d11c2
[txMax,txInd] = max(txFreqs)
txMin = min(txFreqs)
[rxMax,rxMaxInd] = max(rxFreqs)
[rxMin,rxMinInd] = min(rxFreqs)

<<<<<<< HEAD
f1 = abs(mode(beat(rxMinInd:txInd)))
f2 = abs(mode(beat(rxMaxInd:end)))


%% Calculate velocity and distance
=======
f1 = abs(mean(beat(rxMinInd:txInd)))
f2 = abs(mean(beat(rxMaxInd:end)))
>>>>>>> 51b62deef3abaeefd4e0c52b2f63d15a395d11c2

calc_fD = 0.5*(f2-f1);
calc_fB = 0.5*(f2+f1);


<<<<<<< HEAD
calc_vel = abs(calc_fD*(3*10^8)/(2*fc));
calc_dist = calc_fB*(3*10^8)/(2*gamma);
toc
=======
cald_vel = abs(calc_fD*(3*10^8)/(2*fc));
calc_dist = calc_fB*(3*10^8)/(2*gamma);

>>>>>>> 51b62deef3abaeefd4e0c52b2f63d15a395d11c2
