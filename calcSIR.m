clear
close all
load('scen4_wave3.mat', 'beatsignal', 'fs_bs')
% plotBeatSignal(beatsignal, fs_bs, 1)

signal = beatsignal.NoINT;
sigInt = beatsignal.INT;
interferer = beatsignal.INTonly;
fs = fs_bs;
clear beatsignal fs_bs

figure
subplot(211)
hold on
t = (0:(1/fs):(((length(abs(signal)))/fs) - (1/fs)))*1000;
signalDB = 20*log10(abs(signal));
interfererDB = 20*log10(abs(interferer));
sigIntDB = 20*log10(abs(sigInt));
plot(t, signalDB, 'DisplayName', 'Signal only')
plot(t, interfererDB, 'DisplayName', 'Interferer only')
plot(t, sigIntDB, 'DisplayName', 'Signal + Interferer')
hold off
legend('location', 'eastoutside')
xlabel('Time (ms)')
ylabel('dB')
title('Beat Signal Power')

subplot(212)
taps = 1;
plot(t, conv(signalDB-interfererDB, ones(taps,1)/taps, 'same'))
title('SIR')
xlabel('Time (ms)')
ylabel('SIR')
sir = sum(signalDB-interfererDB)./fs;
display(horzcat('SIR is ', num2str(sir), ' dB'))
%%
% sum(signalDB./interfererDB)
% figure
% subplot(211)
% hold on
% t = (0:(1/fs):(((length(abs(signal)))/fs) - (1/fs)))*1000;
% [signalPxx, w] = pwelch(signal,[],[],[], fs, 'centered', 'power');
% interfererPxx = pwelch(interferer, [], [],[],fs, 'centered', 'power');
% sigIntPxx = pwelch(sigInt, [],[],[],fs, 'centered', 'power');
% f = 0:(fs):((length(signal)-1)*fs);
% plot(w, 10*log10(signalPxx), 'DisplayName', 'Signal only')
% plot(w, 10*log10(interfererPxx), 'DisplayName', 'Interferer only')
% plot(w, 10*log10(sigIntPxx), 'DisplayName', 'Signal + Interferer')
% hold off
% legend('location', 'eastoutside')
% ylabel('dB')
% title('Beat Signal Power')
% 
% subplot(212)
% plot(w, pow2db(signalPxx./interfererPxx))
% title('SIR')
% xlabel('freq (hz)')
% ylabel('SIF')
% 
% sirf = sum(pow2db(signalPxx)./pow2db(interfererPxx))/fs
