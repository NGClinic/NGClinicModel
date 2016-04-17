function numIntdlg
close all
f = figure('Position',[500 500 300 100]);

tbLabel = uicontrol(f,'Style','text',...
                'String','Enter Number of Interferers',...
                'FontSize',12,...
                'Position',[30 50 150 30]);
            
tbInput = uicontrol(f,'Style','edit',...
                'String','1',...
                'Position',[190 63 40 20]);
            
pbNext = uicontrol(f,'Style','pushbutton','String','Next',...
                    'FontSize',12,'Position',[160 20 75 30],...
                    'Callback',@pbNext_Callback);
                
pbCancel = uicontrol(f,'Style','pushbutton','String','Cancel',...
                    'FontSize',12,'Position',[75 20 75 30],...
                    'Callback',@pbCancel_Callback);
                
    function pbNext_Callback(source,callbackdata)
       numInt = str2double(get(tbInput,'string'));
       if isnan(numInt)||numInt<0 
           errordlg('You must enter a positive integer')
           return
       else           
           close(f)
           interfererTable(numInt)
       end
    end    
                
    function pbCancel_Callback(source,callbackdata)
       display('Program Cancelled')
       display('------------------');
       close(gcf)
       clear all
    end
end