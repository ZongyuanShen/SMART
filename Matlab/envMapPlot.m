%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Zongyuan Shen %
% All Rights Reserved   %
%%%%%%%%%%%%%%%%%%%%%%%%%

function envMapPlot(staticObsMap, start, goal)
    
    %% Plot the occupancy grid map
    global cellSize;
    map = binaryOccupancyMap(staticObsMap,1/cellSize);
    show(map);
    set(gcf,'color','w');
    title('');
    xlabel('');
    ylabel('');
    set(gca,'XTickLabel','', 'YTickLabel','');
    set(gcf, 'WindowState', 'maximized');
    hold on;
    plot(start(1),start(2),'pentagram','MarkerSize',10,'MarkerEdgeColor','red','MarkerFaceColor','red');
    hold on;
    plot(goal(1),goal(2),'square','MarkerSize',10,'MarkerEdgeColor','blue','MarkerFaceColor','blue');
end