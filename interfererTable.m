function interfererTable(numInt)
    global data targVel targRange
    f = figure('Visible','off');
%     numInt = 6;
    cols = 3;
    rowName = 1:numInt;
    t = uitable('Parent',f,...
                'ColumnName',{'xLoc','yLoc','Velocity'},'RowName',rowName,...
                'Data',zeros(numInt,cols),'FontSize',12,...
                'ColumnEditable', true,...
                'CellEditCallback',@converttonum);  
    pbSim = uicontrol(f,'Style','pushbutton','String','Simulate',...
                    'FontSize',12,'Position',[300 20 75 30],...
                    'Callback',@pbSim_Callback);
    pbCancel = uicontrol(f,'Style','pushbutton','String','Cancel',...
                    'FontSize',12,'Position',[300 60 75 30],...
                    'Callback',@pbCancel_Callback);
    tExtent = get(t,'Extent');
    if tExtent(4) < 100
        figHeight = 100;
    else
        figHeight = round(tExtent(4));
    end
    set(t,'Position',[20 20 round(tExtent(3)) round(tExtent(4))]);
    set(f,'position',[400 400 400 figHeight+40]); 
    
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

    function pbSim_Callback(source,callbackdata)
        display('Running Simulation')
        display('------------------')
        display(targVel)
        display(targRange)
        display(data)
        close(gcf)
    end

    function pbCancel_Callback(source,callbackdata)
        display('Program Cancelled')
        display('------------------')
        close(gcf)
        clear all
        return
    end
end