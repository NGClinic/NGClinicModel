%% Plotting Vehicle Positions
if (PLOT.VEHICLES)
    figure
    radar_pos_x = radar_pos(:,1);
    radar_pos_y = radar_pos(:,2);
    radar_pos_z = radar_pos(:,3);
    tgt_pos_x = tgt_pos(:,1);
    tgt_pos_y = tgt_pos(:,2);
    tgt_pos_z = tgt_pos(:,3);
    int_pos_x = itfer_pos(:,1);
    int_pos_y = itfer_pos(:,2);
    int_pos_z = itfer_pos(:,3);
    hold on
    plot(radar_pos_x,radar_pos_y, 'b-', 'DisplayName','Our Radar');
    plot(radar_pos_x(1),radar_pos_y(1), 'bo', 'DisplayName', 'Start');
    plot(radar_pos_x(Nsweep),radar_pos_y(Nsweep), 'bx', 'DisplayName', 'End');

    plot(tgt_pos_x, tgt_pos_y, 'g-', 'DisplayName', 'Target System');
    plot(tgt_pos_x(1), tgt_pos_y(1), 'go', 'DisplayName', 'Start');
    plot(tgt_pos_x(Nsweep), tgt_pos_y(Nsweep), 'gx', 'DisplayName', 'End');

    plot(int_pos_x, int_pos_y, 'r-', 'DisplayName', 'Interferer System');
    plot(int_pos_x(1), int_pos_y(1), 'ro', 'DisplayName', 'Start');
    plot(int_pos_x(Nsweep), int_pos_y(Nsweep), 'rx', 'DisplayName', 'End');
    xlabel('X (m)')
    ylabel('Y (m)')
    legend('Location', 'eastoutside')
    title('Position of Vehicles')
    grid
   axis([-10 100 -1 5])
    hold off
end