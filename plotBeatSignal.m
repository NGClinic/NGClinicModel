function time = plotBeatSignal( beatSignal, fs, PLOT, MUTUAL_INTERFERENCE)
% plot beat signal
if PLOT
    % Plot signgal without interference
    figure
    subplot(211)
    time = (0:(1/fs):(((length(beatSignal.NoINT))/fs) - (1/fs)))*1000;
    plot(time, real(beatSignal.NoINT),'g')
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
    xlim([-fs/2 fs/2])
    
    
    % Plot signals if mutual interference
    if MUTUAL_INTERFERENCE
        
        % plot signal + interference
        figure
        subplot(211)
        time = (0:(1/fs):(((length(beatSignal.INT))/fs) - (1/fs)))*1000;
        plot(time, real(beatSignal.INT),'k--')
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
        xlim([-fs/2 fs/2])
        
        
        % plot just interference
        figure
        subplot(211)
        time = (0:(1/fs):(((length(beatSignal.INTonly))/fs) - (1/fs)))*1000;
        plot(time, real(beatSignal.INTonly),'r')
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
        xlim([-fs/2 fs/2])
        
        
        % Plot all three signals on top of each other
        figure
        subplot(211)
        time = (0:(1/fs):(((length(beatSignal.NoINT))/fs) - (1/fs)))*1000;
        plot(time, real(beatSignal.NoINT),'g',...
            time, real(beatSignal.INTonly),'r',...
            time, real(beatSignal.INT),'k--');
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
        xlim([-fs/2 fs/2])
    end
    
end
end

