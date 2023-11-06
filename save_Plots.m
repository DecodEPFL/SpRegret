function [] = save_Plots(FolderName, extension)
    mkdir(FolderName)
    FigList = findobj(allchild(0), 'flat', 'Type', 'figure');
    for iFig = 1:length(FigList)
          FigHandle = FigList(iFig);
          FigName   = num2str(get(FigHandle, 'Number'));
          set(0, 'CurrentFigure', FigHandle);
        %   saveas(FigHandle, strcat(FigName, '.png'));
          saveas(FigHandle, fullfile(FolderName,strcat(FigName, extension))); % specify the full path
    end
end

