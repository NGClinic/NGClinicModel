function plotVehiclePositions( radarPos, tgtPos, itferPos,...
     PLOT, MUTUAL_INTERFERENCE,TARGET)
% Initialize max and min of itfer
maxItferX = 0;
minItferX = 0;
maxItferY = 0;
minItferY = 0;

% Plot Vehicles
numInt = size(itferPos,3);
if PLOT
    figure
    hold on
     
    % Plot transciever
    plot(radarPos(:,1),radarPos(:,2), 'g-', 'DisplayName','Our Radar');
    plot(radarPos(1,1),radarPos(1,2), 'go', 'DisplayName', 'Start');
    plot(radarPos(end,1),radarPos(end,2), 'gx', 'DisplayName', 'End');
    legend('Location', 'eastoutside')
    
    % Plot target
    if TARGET
        plot(tgtPos(:,1), tgtPos(:,2), 'm-', 'DisplayName', 'Target System');
        plot(tgtPos(1,1), tgtPos(1,2), 'mo', 'DisplayName', 'Start');
        plot(tgtPos(end,1), tgtPos(end,2), 'mx', 'DisplayName', 'End');
        legend('Location', 'eastoutside')
    end
              
    % Plot interferer(s)
    if MUTUAL_INTERFERENCE
        for i=1:numInt
            plot(itferPos(:,1,i), itferPos(:,2,i), 'r-', 'DisplayName', 'Interferer System');
            plot(itferPos(1,1,i), itferPos(1,2,i), 'ro', 'DisplayName', 'Start');
            plot(itferPos(end,1,i), itferPos(end,2,i), 'rx', 'DisplayName', 'End');
           
            % Calculate range of positions for scaling
            maxItferX = max(maxItferX, max(itferPos(:,1,i)));
            minItferX = min(minItferX, min(itferPos(:,1,i)));
            maxItferY = max(maxItferY, max(itferPos(:,2,i)));
            minItferY = min(minItferY, min(itferPos(:,2,i)));
            
            if i == 1
                legend('Location', 'eastoutside')
            end       
        end
    end
    
    % Label axis and title
    xlabel('X (m)')
    ylabel('Y (m)')
    title('Position of Vehicles')
    xlim([(min(0, minItferX)-5) (max(maxItferX,max(tgtPos(:,1)))+5)])
    ylim([(min(minItferY,min(tgtPos(:,2)))-2) (max(maxItferY,max(tgtPos(:,2)))+2)])
    grid on
    hold off
end
end

