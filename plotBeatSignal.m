function time = plotBeatSignal( beatsignal, fs, PLOT, MUTUAL_INTERFERENCE, TARGET)
% plot beat signal
if PLOT
    
    % Plot signals if mutual interference
    if MUTUAL_INTERFERENCE
        if TARGET
            % Plot all three signals on top of each other
            figure
            subplot(211)
            time = (0:(1/fs):(((length(beatsignal.NoINT))/fs) - (1/fs)))*1000;
            plot(time, real(beatsignal.NoINT),'g',...
                time, real(beatsignal.INTonly),'r',...
                time, real(beatsignal.INT),'k');
            title('Beat Signal')
            xlabel('time(ms)')
            ylabel('Amplitude')
            legend('Signal Only', 'Interference Only', 'Signal + Interference')
            %set(gca, 'xtick', [0:(dur - (1/fs)):(((length(beatSignal))/fs) - (1/fs))].*1000)
            grid on
            ax = gca;

            subplot(212)
            NFFT = 2^nextpow2(length(beatsignal.NoINT));
            f = (((-NFFT/2)):(NFFT/2-1))*(fs/NFFT);
            plot(f, fftshift(abs(fft(beatsignal.NoINT, NFFT))),'g',...
                f, fftshift(abs(fft(beatsignal.INTonly, NFFT))),'r',...
                f, fftshift(abs(fft(beatsignal.INT, NFFT))),'k')
            title('FFT')
            xlabel('freq (Hz)')
            ylabel('|fft|')
            legend('Signal Only', 'Interference Only', 'Signal + Interference')
            xlim([-3000 3000])
            grid on



            % plot signal + interference
            figure
            subplot(211)
            time = (0:(1/fs):(((length(beatsignal.INT))/fs) - (1/fs)))*1000;
            plot(time, real(beatsignal.INT),'k')
            title('Beat Signal with Interference')
            xlabel('time(ms)')
            ylabel('Amplitude')
            %set(gca, 'xtick', [0:(dur - (1/fs)):(((length(beatSignal))/fs) - (1/fs))].*1000)
            grid on
            ylim(ax.YLim);

            subplot(212)
            NFFT = 2^nextpow2(length(beatsignal.INT));
            f = (((-NFFT/2)):(NFFT/2-1))*(fs/NFFT);
            plot(f, fftshift(abs(fft(beatsignal.INT, NFFT))),'k')
            title('FFT')
            xlabel('freq (Hz)')
            ylabel('|fft|')
            xlim([-3000 3000])
            grid on
        end
        
        % plot just interference
        figure
        subplot(211)
        time = (0:(1/fs):(((length(beatsignal.INTonly))/fs) - (1/fs)))*1000;
        plot(time, real(beatsignal.INTonly),'r')
        title('Beat Signal only Interference')
        xlabel('time(ms)')
        ylabel('Amplitude')
        %set(gca, 'xtick', [0:(dur - (1/fs)):(((length(beatSignal))/fs) - (1/fs))].*1000)
        grid on
        if TARGET 
            ylim(ax.YLim);
        end
        
        subplot(212)
        NFFT = 2^nextpow2(length(beatsignal.INTonly));
        f = (((-NFFT/2)):(NFFT/2-1))*(fs/NFFT);
        plot(f, fftshift(abs(fft(beatsignal.INTonly, NFFT))),'r')
        title('FFT')
        xlabel('freq (Hz)')
        ylabel('|fft|')
        xlim([-3000 3000])  
        grid on
        
    end
    
    if TARGET
        % Plot signgal without interference
        figure
        subplot(211)
        time = (0:(1/fs):(((length(beatsignal.NoINT))/fs) - (1/fs)))*1000;
        plot(time, real(beatsignal.NoINT),'g')
        title('Beat Signal without Interference')
        xlabel('time(ms)')
        ylabel('Amplitude')
        %set(gca, 'xtick', [0:(dur - (1/fs)):(((length(beatSignal))/fs) - (1/fs))].*1000)
        grid on
        ylim(ax.YLim);
        grid on

        subplot(212)
        NFFT = 2^nextpow2(length(beatsignal.NoINT));
        f = (((-NFFT/2)):(NFFT/2-1))*(fs/NFFT);
        xfft = fftshift(abs(fft(beatsignal.NoINT, NFFT)));
        plot(f, xfft,'g')
        title('FFT')
        xlabel('freq (Hz)')
        ylabel('|fft|')
        xlim([-3000 3000])
        grid on
    end
    
end
end

