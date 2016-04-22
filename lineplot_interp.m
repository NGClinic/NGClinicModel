
clear all
close all

% cut time :
% 30m,1 : 7.13
% 30m,2 : 

%[rangeline,lowtol,hightol,time] = lineplot('Scenario3\scen3_wave1_30m_1.wav',7.13);
[rangeline,lowtol,hightol,time] = lineplot('Scenario3\scen3_30m_off_1.wav',4);

% xq = linspace(25.75,10.11,2*length(rangeline));
% interprange = interp1(time(1:end-1),rangeline,xq);
% figure(1)
% plot(xq,interprange)

figure(3)
mdl = fitlm(time(1:end-1),rangeline,'linear');
plot(mdl)


