function [tx_signal, reflected_signal,fdoppler, tshift] = object_processing( df, fc, fs, Tm, theta, distance, velocity)
%%-------------------------------------------------------------------------
% Chirp Generator Function
% mngai@hmc.edu 
%%-------------------------------------------------------------------------

% Generate Chirp
fmin = fc-df/2;             % Hz
fmax = fc+df/2;             % Hz
dt = 1/fs;                  % Time Step (s)
t = 0:dt:2*Tm;              % Duration of chirp (s)

% Generate signal
ramp_gen = sawtooth(2*pi*t/(2*Tm),0.5);
tx_signal = vco(ramp_gen,[fmin fmax],fs);

spectrogram(tx_signal,256*2,220*2,512*8,fs,'yaxis')
xlabel('Time(s)')
ylabel('Frequency (Hz)')
title('Spectrogram of Generated Chirp')


%% Object Processing
% Time Shift
c = 3*10^8;                 % speed of light (m/s)
tdelay = 2*distance/c;      % time delay (s)
tshift = (tdelay/dt)*time_factor;

% Doppler Shift
vr = velocity*sin(theta);   % radial velocity (m/s)
lambda = c/fc;              % wavelength of carrier signal (m)
fdoppler = ((2*vr)/lambda); % Doppler shift (Hz)


%% Apply Freq and Time Shift

% % Determine the necessary resolution for the fft shift
% fftres = fdoppler;
% fftsize = 2^nextpow2(fs/fftres);
% fftres = fs/fftsize;


% Apply freq shift
fftsize = 2^nextpow2(length(tx_signal));
fftres = fs/fftsize;
fftshift = round(fdoppler/fftres);
fdoppler = fftshift*fftres;
fft_tx = fft(tx_signal,fftsize);
figure
plot(0:fs/fftsize:fs-fs/fftsize, abs(fft_tx));
    
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
reflected_signal = circshift(tx_freqshifted, [0, round(tshift/dt)]);
figure
spectrogram(reflected_signal,256*2,220*2,512*8,fs,'yaxis')
xlabel('Time(s)')
ylabel('Frequency (Hz)')
title('Spectrogram of Reflected Chirp')
axis([0 Tm*2 fmin*.6 fmax*1.4])



end