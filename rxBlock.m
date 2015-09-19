function [ dist, vel, f_D, f_B ] = rxBlock( tx_signal, reflected_signal, gamma, df )

fs = 11800;
[chirpRefl,freqsRefl,t] = spectrogram(reflected_signal,256*2,220*2,512*8,fs);
[chirpIn,freqIn,tin] = spectrogram(tx_signal,256*2,220*2,512*8,fs);
figure
plot(freqIn,chirpIn(:,2))
figure

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

figure
plot(rxFreqs, 'r')
hold on
plot(txFreqs, 'b')
legend('rxFreqs', 'txFreqs')
hold off

beat = abs(rxFreqs - txFreqs);

figure
plot(beat)
title('beat')

[txMax,txInd] = max(txFreqs);
txMin = min(txFreqs);
[rxMax,rxMaxInd] = max(rxFreqs);
[rxMin,rxMinInd] = min(rxFreqs);

f1 = abs(mean(beat(rxMinInd:txInd)))
f2 = abs(mean(beat(rxMaxInd:end)))

f_D = 0.5*(f2-f1);
f_B = 0.5*(f2+f1);


vel = abs(f_D*(3*10^8)/(2*df));
dist = f_B*(3*10^8)/(2*gamma);

end
