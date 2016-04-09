function plotBeatSignal( beatSignal, fs, name )
% plot beat signal
    figure
    subplot(211)
    dur = length(beatSignal(:,1))/fs;
    bs_t = (0:(1/fs):(((length(beatSignal))/fs) - (1/fs)))*1000;
    plot(bs_t, abs(beatSignal))
    title(name)
    xlabel('time(ms)')
    ylabel('Amplitude')
    %set(gca, 'xtick', [0:(dur - (1/fs)):(((length(beatSignal))/fs) - (1/fs))].*1000)
    grid on

    subplot(212)
    NFFT = 2^nextpow2(length(beatSignal));
    f = (((-NFFT/2)):(NFFT/2-1))*(fs/NFFT);
    plot(f, fftshift(abs(fft(beatSignal, NFFT))))
    title('FFT')
    xlabel('freq (Hz)')
    ylabel('|fft|')
    xlim([-4000 4000])

end

