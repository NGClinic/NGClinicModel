clc
clear
tic
temp = zeros(1,5);
for i = 1:5
    filename = horzcat('Scenario3\scen3_wave1_05m_on_', num2str(i), '.wav');
    temp(i) = calcPower(filename);
    toc
end
average05 = mean(temp);
dB05 = 10*log10(average05)

tic
temp = zeros(1,5);
for i = 1:5
    filename = horzcat('Scenario3\scen3_wave1_10m_on_', num2str(i), '.wav');
    temp(i) = calcPower(filename);
    toc
end
average10 = mean(temp);
dB10 = 10*log10(average10)


tic
temp = zeros(1,5);
for i = 1:5
    filename = horzcat('Scenario3\scen3_wave1_15m_on_', num2str(i), '.wav');
    temp(i) = calcPower(filename);
    toc
end
average15 = mean(temp);
dB15 = 10*log10(average15)

    