function [radarPos, tgtPos, itferPos,...
    radarVel, tgtVel, itferVel] = prevEnv( Nsweep, tm,...
    radar_init_pos, tgt_init_pos, itfer_init_pos,...
    radar_speed_x, tgt_speed_x, itfer_speed_x, PLOT, MUTUAL_INTERFERENCE)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here


t = (0:Nsweep-1)*(tm/2);
len = length(t);
radarPosX = radar_speed_x.*t + radar_init_pos(1);
tgtPosX = tgt_speed_x.*t + tgt_init_pos(1);
itferPosX = itfer_speed_x.*t + itfer_init_pos(1);

radarPos = [radarPosX
             radar_init_pos(2)*ones(1,len)
             radar_init_pos(3)*ones(1,len)]';
tgtPos = [tgtPosX
             tgt_init_pos(2)*ones(1,len)
             tgt_init_pos(3)*ones(1,len)]';
itferPos = [itferPosX
             itfer_init_pos(2)*ones(1,len)
             itfer_init_pos(3)*ones(1,len)]';
radarVel = [radar_speed_x*ones(1,len)
            zeros(1,len); zeros(1,len)]';
tgtVel = [tgt_speed_x*ones(1,len)
            zeros(1,len); zeros(1,len)]';
itferVel = [itfer_speed_x*ones(1,len)
            zeros(1,len); zeros(1,len)]';
plotVehiclePositions( radarPos, tgtPos, itferPos,...
     PLOT, MUTUAL_INTERFERENCE)
 
end

