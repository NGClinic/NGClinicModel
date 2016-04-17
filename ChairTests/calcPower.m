function P3off = calcPower(filename)
% Cross-correlation of different tests

[Y3off,FS] = audioread(filename);

%constants
c = 3E8; %(ms) speed of light
%radar parameters
Tp = 20E-3; %(s) pulse time
N = Tp*FS; %# of samples per pulse
fstart = 2260E6; %(Hz) LFM start frequency for example
fstop = 2590E6; %Hz) LFM stop frequency for example
%fstart = 2402E6; %(Hz) LFM start frequency for ISM band
%fstop = 2495E6; %(Hz) LFM stop frequency for ISM band
BW = fstop-fstart; %(Hz) transmit bandwidth
f = linspace(fstart, fstop, N/2); %instantaneous transmit frequency

%range resolution
rr = c/(2*BW);
max_range = rr*N/2;

%the input appears to be inverted
trig3off = -1*Y3off(:,1);
s_3off = -1*Y3off(:,2);

%parse the data here by triggering off rising edge of sync pulse
count = 0;
thresh = 0;
start_3off = (trig3off > thresh); % Same size, either 1 (true) or 0 (false)

for ii = 100:(size(start_3off,1)-N)
    if start_3off(ii) == 1 && mean(start_3off(ii-11:ii-1)) == 0
        start2_3off(ii:ii+N-1) = s_3off(ii:ii+N-1);
        count = count + 1;
        sif_3off(count,:) = s_3off(ii:ii+N-1);
        % each row of sif is N data (one pulse)
        time_3off(count) = ii*1/FS;
        % corresponding vector of time 
    end
end

% Total Signal Power in Single Value
P3off = rms(reshape(sif_3off,1,[]))^2;


end

