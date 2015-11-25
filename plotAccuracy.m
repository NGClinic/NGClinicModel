if (PLOT.ACCURACY)
    figure
    subplot(211);
    suptitle(['Accuracy with ' num2str(Nsweep) ' Sweeps'])
    plot((1:Nsweep)*hwav.SweepTime*hwav.NumSweeps, rng_true, ...
        '.-', 'DisplayName', 'Target Range (m)')
    hold on
    plot((1:Nsweep)*hwav.SweepTime*hwav.NumSweeps, rng_est, ...
        '.-', 'DisplayName', 'Calculated Range (m)');
    legend('Location', 'eastoutside'); title('Range'); ylabel('m'); xlabel('s');
    
    subplot(212);
    plot((1:Nsweep)*hwav.SweepTime*hwav.NumSweeps, ...
       (radar_speed-car_speed)*ones(Nsweep,1), ...
        '.-', 'DisplayName', 'Target Speed (m/s)')
    hold on
    plot((1:Nsweep)*hwav.SweepTime*hwav.NumSweeps, v_est,...
        '.-', 'DisplayName', 'Calculated Speed (m/s)');
    hold off
    legend('Location', 'eastoutside')
    title('Velocity'); ylabel('m/s');  xlabel('s');
end