function interfererTable(numInt)
    global fc tm bw radar_speed bw_INT tm_INT
    global txPower txLossFactor rxNF rxLossFactor rad_pat
    global radar_init_pos car_init_pos car_speed targRange
    global itfer_speed itfer_init_pos
    global Nsweep scenType
    global data
    
    f = figure('Visible','off');
%     numInt = 6;
    cols = 3;
    rowName = 1:numInt;
    tableDat = zeros(numInt,cols);
    tableDat(1,1) = 10;
    tableDat(1,2) = 3.048;
    t = uitable('Parent',f,...
                'ColumnName',{'xLoc','yLoc','Velocity'},'RowName',rowName,...
                'Data',tableDat,'FontSize',12,...
                'ColumnEditable', true,...
                'CellEditCallback',@converttonum);
            
    pbPreview = uicontrol(f,'Style','pushbutton','String','Preview',...
                    'FontSize',12,'Position',[300 100 75 30],...
                    'Callback',@pbPreview_Callback);         
            
    pbSim = uicontrol(f,'Style','pushbutton','String','Simulate',...
                    'FontSize',12,'Position',[300 60 75 30],...
                    'Callback',@pbSim_Callback);                          
                
    pbCancel = uicontrol(f,'Style','pushbutton','String','Cancel',...
                    'FontSize',12,'Position',[300 20 75 30],...
                    'Callback',@pbCancel_Callback);
    tExtent = get(t,'Extent');
    if tExtent(4) < 100
        figHeight = 100;
    else
        figHeight = round(tExtent(4));
    end
    set(t,'Position',[20 20 round(tExtent(3)) round(tExtent(4))]);
    set(f,'position',[400 400 400 figHeight+40]); 
    
%     radar_init_pos = [0;0;0.5];
    rangeMax = 80;
    PLOT.VEHICLES = 1;
    PLOT.POWER = 0;
    PLOT.ACCURACY = 0;
    PLOT.BEATSIGNAL = 0;
    PLOT.PREVIEW = 0;
    PLOT.CHIRP = 0;
    MUTUAL_INTERFERENCE= 1;
    TARGET = 1;
    SAVE = 0;
    PHASE_SHIFT = 0;
    fileName = 'filename.mat';
    LPmixer = 28e3;
    tm = 10e-3;
    tm_INT = tm;
    target = 'car';
    
    f.Visible = 'on';
    
    % Callback functions
    function converttonum(t,callbackdata)
        r = callbackdata.Indices(1);
        c = callbackdata.Indices(2);
        data = get(t,'Data');
        if isnan(data(r,c))
            data(r,c) = 0;
            set(t,'Data',data)
        end       
    end

    function pbPreview_Callback(source,callbackdata)
        itferData = get(t,'Data');
        
        prevEnv(Nsweep, tm, radar_init_pos, car_init_pos, itferData,...
                 radar_speed, car_speed,...
                 1, 1, TARGET)
        display(itferData)
    end

    function pbSim_Callback(source,callbackdata)
        data = get(t,'Data');
        itfer_speed = data(1,3);
        itfer_init_pos = [data(1,1) data(1,2) 0.5];
        display('Running Simulation')
        display('------------------')
        load('SampleRadiationPatterns.mat', 'TPLink');
        rad_pat = TPLink; clear TPLink;
        close(gcf)
        
        [radarPos, tgtPos, itferPos,...
        radarVel, tgtVel, itferVel] = prevEnv( Nsweep, tm,...
            radar_init_pos, car_init_pos, itfer_init_pos,...
            radar_speed, car_speed, itfer_speed, PLOT.PREVIEW,...
            MUTUAL_INTERFERENCE, TARGET);
        
        [~, beatsignal, fs_bs] = radarSim(fc, tm, tm_INT, rangeMax, bw,...
            bw_INT, Nsweep, LPmixer, rad_pat, radarPos,...
            itferPos, tgtPos, radarVel, itferVel,...
            tgtVel, txPower, txLossFactor,rxNF, rxLossFactor,...
            PLOT, MUTUAL_INTERFERENCE,TARGET, ...
            PHASE_SHIFT, SAVE, fileName, target);
        [output] = calcSimSIR(beatsignal, fs_bs)
    end

    function pbCancel_Callback(source,callbackdata)
        display('Program Cancelled')
        display('------------------')
        close(gcf)
        clear all
        return
    end
end