function [radarPos, tgtPos, itferPos,...
    radarVel, tgtVel, itferVel] = prevEnv(Nsweep, tm,...
    radar_init_pos, tgt_init_pos, itferData,...
    radar_speed_x, tgt_speed_x, PLOT, MUTUAL_INTERFERENCE, TARGET)
%preEnv
%   inputs the initial position and velocity of the transceiver, target, and
%   interferer(s).
%
%   output position path an velocity of transceiver, target, and
%   interferer(s)
%
%   also plots the path of the vehicles

t = (0:Nsweep-1)*(tm/2);
len = length(t);

% Calculates Radar position and velocity
radarPosX = radar_speed_x.*t + radar_init_pos(1);
radarPos = [radarPosX
             radar_init_pos(2)*ones(1,len)
             radar_init_pos(3)*ones(1,len)]';
radarVel = [radar_speed_x*ones(1,len)
            zeros(1,len); zeros(1,len)]';

% Calculate target positionand velocity
tgtPosX = tgt_speed_x.*t + tgt_init_pos(1);
tgtPos = [tgtPosX
             tgt_init_pos(2)*ones(1,len)
             tgt_init_pos(3)*ones(1,len)]';
tgtVel = [tgt_speed_x*ones(1,len)
    zeros(1,len); zeros(1,len)]';

% Initiates interferer position and velocity
numInt = size(itferData,1);
itferPos = zeros(size(tgtPos,1), size(tgtPos,2), numInt);
itferVel = zeros(size(tgtVel,1), size(tgtVel,2), numInt);

% Calculate interferer position and velocity
if MUTUAL_INTERFERENCE
    for i=1:numInt
            % Get position and velocity data for interferer
            itferPosX = itferData(i,3).*t + itferData(i,1);
            itferPos(:,:,i) = [itferPosX
                 itferData(i,2)*ones(1,len)
                 0.5*ones(1,len)]';
            itferVel(:,:,1) = [itferData(i,3)*ones(1,len)
                        zeros(1,len); zeros(1,len)]';
    end
end

plotVehiclePositions( radarPos, tgtPos, itferPos,...
     PLOT, MUTUAL_INTERFERENCE,TARGET)
end

