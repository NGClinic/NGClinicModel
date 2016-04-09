function plotVehiclePositions( radar_pos, tgt_pos, itfer_pos,...
    MUTUAL_INTERFERENCE, PLOT)
% plot vehicle positions
if PLOT
    figure
    hold on

    if MUTUAL_INTERFERENCE
        plot(itfer_pos(:,1), itfer_pos(:,2), 'r-', 'DisplayName', 'Interferer System');
        plot(itfer_pos(1,1), itfer_pos(1,2), 'ro', 'DisplayName', 'Start');
        plot(itfer_pos(end,1), itfer_pos(end,2), 'rx', 'DisplayName', 'End');
    end
        
    plot(radar_pos(:,1),radar_pos(:,2), 'g-', 'DisplayName','Our Radar');
    plot(radar_pos(1,1),radar_pos(1,2), 'go', 'DisplayName', 'Start');
    plot(radar_pos(end,1),radar_pos(end,2), 'gx', 'DisplayName', 'End');

    plot(tgt_pos(:,1), tgt_pos(:,2), 'k-', 'DisplayName', 'Target System');
    plot(tgt_pos(1,1), tgt_pos(1,2), 'ko', 'DisplayName', 'Start');
    plot(tgt_pos(end,1), tgt_pos(end,2), 'kx', 'DisplayName', 'End');
    
    
    xlabel('X (m)')
    ylabel('Y (m)')
    legend('Location', 'eastoutside')
    title('Position of Vehicles')
    zoom out
    grid
    hold off
end
end

