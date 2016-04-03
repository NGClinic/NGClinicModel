function interpolateAntennaPattern( antenna, name )
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
azdB = antenna.azdB;
xx = min(az):1:max(az);
yy = spline(az, azdB, xx);
plot(az,azdB, 'o', xx,yy)
axis([min(az) max(az) (min(azdB)-5) (max(azdB)+5)])
grid on
xlabel('Azimuth (deg.)')
ylabel('Directivity (dBi)')
title('Array Directivity Variation-Elevation  = 0 deg.')

subplot(212)
el = antenna.el;
eldB = antenna.eldB;
xx = min(el):1:max(el);
yy = spline(el, eldB, xx);
plot(el,eldB, 'o', xx,yy)
axis([min(el) max(el) (min(eldB)-5) (max(eldB) + 5)])
grid on
xlabel('Elevation (deg.)')
ylabel('Directivity (dBi)')
title('Array Directivity Variation-Azimuth = 0 deg.')
suptitle(name)
end

