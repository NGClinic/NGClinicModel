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


% % Initialize max and min of itfer
% maxItferX = 0;
% minItferX = 0;
% maxItferY = 0;
% minItferY = 0;
% 
% % Plot Vehicles
% numInt = size(itferData,1);
% if PLOT
%     figure
%     hold on
%      
%     % Plot transciever
%     plot(radarPos(:,1),radarPos(:,2), 'g-', 'DisplayName','Our Radar');
%     plot(radarPos(1,1),radarPos(1,2), 'go', 'DisplayName', 'Start');
%     plot(radarPos(end,1),radarPos(end,2), 'gx', 'DisplayName', 'End');
%     legend('Location', 'eastoutside')
%     
%     % Plot target
%     if TARGET
%         plot(tgtPos(:,1), tgtPos(:,2), 'm-', 'DisplayName', 'Target System');
%         plot(tgtPos(1,1), tgtPos(1,2), 'mo', 'DisplayName', 'Start');
%         plot(tgtPos(end,1), tgtPos(end,2), 'mx', 'DisplayName', 'End');
%         legend('Location', 'eastoutside')
%     end
%               
%     % Plot interferer(s)
%     if MUTUAL_INTERFERENCE
%         for i=1:numInt
%             plot(itferPos(:,1,i), itferPos(:,2,i), 'r-', 'DisplayName', 'Interferer System');
%             plot(itferPos(1,1,i), itferPos(1,2,i), 'ro', 'DisplayName', 'Start');
%             plot(itferPos(end,1,i), itferPos(end,2,i), 'rx', 'DisplayName', 'End');
%            
%             % Calculate range of positions for scaling
%             maxItferX = max(maxItferX, max(itferPosX));
%             minItferX = min(minItferX, min(itferPosX));
%             maxItferY = max(maxItferY, max(itferPos(:,2,i)));
%             minItferY = min(minItferY, min(itferPos(:,2,i)));
%             
%             if i == 1
%                 legend('Location', 'eastoutside')
%             end       
%         end
%     end
%     
%     % Label axis and title
%     xlabel('X (m)')
%     ylabel('Y (m)')
%     title('Position of Vehicles')
%     xlim([(min(0, minItferX)-5) (max(maxItferX,max(tgtPos(:,1)))+5)])
%     ylim([(min(minItferY,min(tgtPos(:,2)))-2) (max(maxItferY,max(tgtPos(:,2)))+2)])
%     grid on
%     hold off
% end
plotVehiclePositions( radarPos, tgtPos, itferPos,...
     PLOT, MUTUAL_INTERFERENCE,TARGET)
end

