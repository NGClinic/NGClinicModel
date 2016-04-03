function plotAntennaPattern( antenna, name )
% antenna - structure with four components
%     antenna.az
%     antenna.azdB
%     antenna.el
%     antenna.eldB
%     
% name - string with the name of the antenna
figure
subplot(211)
az = antenna.az;
axdB = antenna.azdB;
plot(az,axdB)
axis([min(az) max(az) (min(axdB)-5) (max(axdB)+5)])
grid on
xlabel('Azimuth (deg.)')
ylabel('Directivity (dBi)')
title('Array Directivity Variation-Elevation  = 0 deg.')

subplot(212)
el = antenna.el;
eldB = antenna.eldB;
plot(el,eldB)
axis([min(el) max(el) (min(eldB)-5) (max(eldB) + 5)])
grid on
xlabel('Elevation (deg.)')
ylabel('Directivity (dBi)')
title('Array Directivity Variation-Azimuth = 0 deg.')
suptitle(name)
end

