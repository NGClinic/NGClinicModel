function [radarPos, tgtPos, itferPos,...
    radarVel, tgtVel, itferVel] = prevEnv(Nsweep, tm,...
    radar_init_pos, tgt_init_pos, itferData,...
    radar_speed_x, tgt_speed_x, PLOT, MUTUAL_INTERFERENCE, TARGET)
%UNTITLED2 Summary of this function goes here
%   Currently only returns the last itfer position and velocity


t = (0:Nsweep-1)*(tm/2);
len = length(t);

% Calculates Radar Position and Velocity in X Direction
radarPosX = radar_speed_x.*t + radar_init_pos(1);
tgtPosX = tgt_speed_x.*t + tgt_init_pos(1);

% Calculates Radar and Target Position and Velocity Vector
radarPos = [radarPosX
             radar_init_pos(2)*ones(1,len)
             radar_init_pos(3)*ones(1,len)]';
tgtPos = [tgtPosX
             tgt_init_pos(2)*ones(1,len)
             tgt_init_pos(3)*ones(1,len)]';
itferPos = zeros(size(tgtPos));
radarVel = [radar_speed_x*ones(1,len)
            zeros(1,len); zeros(1,len)]';
tgtVel = [tgt_speed_x*ones(1,len)
    zeros(1,len); zeros(1,len)]';
itferVel = zeros(size(tgtVel));

       
         
% itferPosX = size(numInt, len);
% for i=1:numInt
%     itferPosX(i,:) = itferData(i,3).*t + itferData(i,1);
% end
% itferPosX = itfer_speed_x.*t + itfer_init_pos(1);

% itferPos = [itferPosX
%              itfer_init_pos(2)*ones(1,len)
%              itfer_init_pos(3)*ones(1,len)]';
% itferVel = [itfer_speed_x*ones(1,len)
%             zeros(1,len); zeros(1,len)]';
numInt = size(itferData,1);
maxItferX = 0;
maxItferY = 0;
minItferY = 0;
% Plot Vehicles
if PLOT
    figure
    hold on
     
    plot(radarPos(:,1),radarPos(:,2), 'g-', 'DisplayName','Our Radar');
    plot(radarPos(1,1),radarPos(1,2), 'go', 'DisplayName', 'Start');
    plot(radarPos(end,1),radarPos(end,2), 'gx', 'DisplayName', 'End');
    legend('Location', 'eastoutside')
    
    if TARGET
        plot(tgtPos(:,1), tgtPos(:,2), 'k-', 'DisplayName', 'Target System');
        plot(tgtPos(1,1), tgtPos(1,2), 'ko', 'DisplayName', 'Start');
        plot(tgtPos(end,1), tgtPos(end,2), 'kx', 'DisplayName', 'End');
        legend('Location', 'eastoutside')
    end
    
    
    if MUTUAL_INTERFERENCE
        for i=1:numInt
            % Get position and velocity data for interferer
            itferPosX = itferData(i,3).*t + itferData(i,1);
            itferPos = [itferPosX
                 itferData(i,2)*ones(1,len)
                 0.5*ones(1,len)]';
            itferVel = [itferData(i,3)*ones(1,len)
                        zeros(1,len); zeros(1,len)]';
                    
            plot(itferPos(:,1), itferPos(:,2), 'r-', 'DisplayName', 'Interferer System');
            plot(itferPos(1,1), itferPos(1,2), 'ro', 'DisplayName', 'Start');
            plot(itferPos(end,1), itferPos(end,2), 'rx', 'DisplayName', 'End');
           
            if i == 1
                legend('Location', 'eastoutside')
            end
            
            maxItferX = max(maxItferX, max(itferPosX));
            maxItferY = max(maxItferY, max(itferPos(:,2)));
            minItferY = min(minItferY, min(itferPos(:,2)));
        end
    end
    
    xlabel('X (m)')
    ylabel('Y (m)')
    title('Position of Vehicles')
%     zoom out
    xlim([-5 (max(maxItferX,max(tgtPos(:,1)))+5)])
    ylim([(min(minItferY,min(tgtPos(:,2)))-2) (max(maxItferY,max(tgtPos(:,2)))+2)])
    grid on
    hold off
end
         

 
end

