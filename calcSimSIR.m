% Calculate the SIR for Software Results
function [output] = calcSimSIR(beatsignal, fs_bs)
%close all
% load('scen23_wave1_05m.mat', 'beatsignal', 'fs_bs')
% plotBeatSignal(beatsignal, fs_bs, 1)

signal = beatsignal.NoINT;
sigInt = beatsignal.INT;
interferer = beatsignal.INTonly;
fs = fs_bs;
clear beatsignal fs_bs
%%
tic
signalP = rms(signal).^2;
interfererP = rms(interferer).^2;
sigIntP = rms(sigInt)^2;
output.signalDB = 10*log10(signalP);
output.interfererDB = 10*log10(interfererP);
output.sigIntDB = 10*log10(sigIntP);
output.sirDirect = 10*log10(signalP/interfererP);
output.sirIndirect = 10*log10((sigIntP - interfererP)/(sigIntP - signalP));
% display(horzcat('SIR is ', num2str(output.sirDirect), ' dB'))
% display(horzcat('SIR is ', num2str(output.sirIndirect), ' dB'))

end
