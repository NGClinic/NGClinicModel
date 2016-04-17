function beatSignalPower = plotBeatSignal( beatSignal, fs, PLOT, MUTUAL_INTERFERENCE)
if PLOT% plot beat signal
    figure
    subplot(211)
    bs_t = (0:(1/fs):(((length(beatSignal.NoINT))/fs) - (1/fs)))*1000;
    plot(bs_t, abs(beatSignal.NoINT))
    title('Beat Signal without Interference')
    xlabel('time(ms)')
    ylabel('Amplitude')
    %set(gca, 'xtick', [0:(dur - (1/fs)):(((length(beatSignal))/fs) - (1/fs))].*1000)
    grid on

    subplot(212)
    NFFT = 2^nextpow2(length(beatSignal.NoINT));
    f = (((-NFFT/2)):(NFFT/2-1))*(fs/NFFT);
    xfft = fftshift(abs(fft(beatSignal.NoINT, NFFT)));
    plot(f, xfft)
    title('FFT')
    xlabel('freq (Hz)')
    ylabel('|fft|')
    xlim([-4000 4000])
    
    if MUTUAL_INTERFERENCE
        figure
        subplot(211)
        bs_t = (0:(1/fs):(((length(beatSignal.INT))/fs) - (1/fs)))*1000;
        plot(bs_t, abs(beatSignal.INT))
        title('Beat Signal with Interference')
        xlabel('time(ms)')
        ylabel('Amplitude')
        %set(gca, 'xtick', [0:(dur - (1/fs)):(((length(beatSignal))/fs) - (1/fs))].*1000)
        grid on

        subplot(212)
        NFFT = 2^nextpow2(length(beatSignal.INT));
        f = (((-NFFT/2)):(NFFT/2-1))*(fs/NFFT);
        plot(f, fftshift(abs(fft(beatSignal.INT, NFFT))))
        title('FFT')
        xlabel('freq (Hz)')
        ylabel('|fft|')
        xlim([-4000 4000])
        
        figure
        subplot(211)
        bs_t = (0:(1/fs):(((length(beatSignal.INTonly))/fs) - (1/fs)))*1000;
        plot(bs_t, abs(beatSignal.INTonly))
        title('Beat Signal only Interference')
        xlabel('time(ms)')
        ylabel('Amplitude')
        %set(gca, 'xtick', [0:(dur - (1/fs)):(((length(beatSignal))/fs) - (1/fs))].*1000)
        grid on

        subplot(212)
        NFFT = 2^nextpow2(length(beatSignal.INTonly));
        f = (((-NFFT/2)):(NFFT/2-1))*(fs/NFFT);
        plot(f, fftshift(abs(fft(beatSignal.INTonly, NFFT))))
        title('FFT')
        xlabel('freq (Hz)')
        ylabel('|fft|')
        xlim([-4000 4000])
        
        
        figure
        subplot(211)
        bs_t = (0:(1/fs):(((length(beatSignal.NoINT))/fs) - (1/fs)))*1000;
        plot(bs_t, abs(beatSignal.NoINT),'g',...
            bs_t, abs(beatSignal.INTonly),'r',...
            bs_t, abs(beatSignal.INT),'k');
        title('Beat Signal')
        xlabel('time(ms)')
        ylabel('Amplitude')
        legend('Signal Only', 'Interference Only', 'Signal + Interference')
        %set(gca, 'xtick', [0:(dur - (1/fs)):(((length(beatSignal))/fs) - (1/fs))].*1000)
        grid on

        subplot(212)
        NFFT = 2^nextpow2(length(beatSignal.NoINT));
        f = (((-NFFT/2)):(NFFT/2-1))*(fs/NFFT);
        plot(f, fftshift(abs(fft(beatSignal.NoINT, NFFT))),'g',...
            f, fftshift(abs(fft(beatSignal.INTonly, NFFT))),'r',...
            f, fftshift(abs(fft(beatSignal.INT, NFFT))),'k')
        title('FFT')
        xlabel('freq (Hz)')
        ylabel('|fft|')
        legend('Signal Only', 'Interference Only', 'Signal + Interference')
        xlim([-4000 4000])
    end
    
end
end

