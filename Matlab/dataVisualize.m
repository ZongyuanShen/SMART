%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Zongyuan Shen %
% All Rights Reserved   %
%%%%%%%%%%%%%%%%%%%%%%%%%

function  [h_robot, h_obs, h_tree, h_path] ...
    = dataVisualize(h_robot, h_obs, h_tree, h_path,...
    dynaObs_i, tree_i, path_i, footPrint_i, i)
    
    %% Setup parameters
    global LRZ_radius OHZ_radius obstacleRadius;
    edgeColor_add = rand(20000,3);
    edgeColor = [
    0,0,204;
    0,204,0;
    0,204,204;
    204,0,204;
    204,102,0;
    128,128,0;
    0,100,0;
    100,149,237;
    102,0,204;
    128,0,128;
    255,20,147;
    112,128,144]/255; 
    edgeColor = [edgeColor;edgeColor_add];
    grey = [224 224 224]/255;
    nodeMarkerSize = 1.5;
    lineWidth = 1;
    pathLineWidth = 2;

    %% Plot tree %%
    for k=1:height(tree_i)
        h_tree{i}(1,k) = plot([tree_i(k,1),tree_i(k,3)],[tree_i(k,2),tree_i(k,4)],'Marker','o','MarkerSize',nodeMarkerSize,'LineWidth',lineWidth);
        if tree_i(k,6) == 0
            h_tree{i}(1,k).Color = grey;
        else
            h_tree{i}(1,k).Color = edgeColor(tree_i(k,6),:);
        end
    end

    %% Plot dynamic obstacle %%
    for k=1:height(dynaObs_i)
        if dynaObs_i(k,3) == 1
            h_obs{i}(1,k)=circles(dynaObs_i(k,1),dynaObs_i(k,2),OHZ_radius,'edgecolor','red','facecolor','none'); % Mark CPR
        else
            h_obs{i}(1,k)=circles(dynaObs_i(k,1),dynaObs_i(k,2),OHZ_radius,'edgecolor','black','facecolor','none');
        end
        h_obs{i}(2,k)=circles(dynaObs_i(k,1),dynaObs_i(k,2),obstacleRadius,'edgecolor','black','facecolor',grey,'facealpha',0.6);
    end

    %% Plot path %%
    if isempty(path_i)
        h_path{i} = [];
    else
        path_i = [path_i;[footPrint_i(1,1),footPrint_i(1,2)]];
        for k = height(path_i):-1:2
            h_path{i}(1,k) = plot([path_i(k,1),path_i(k-1,1)],[path_i(k,2),path_i(k-1,2)],'LineWidth',pathLineWidth);
            h_path{i}(1,k).Color = 'red';
        end 
    end

    %% Plot robot position %%
    h_robot{i}(1,1) = circles(footPrint_i(1,1),footPrint_i(1,2), LRZ_radius,'edgecolor','green','facecolor','green','facealpha',0.0,'LineWidth',pathLineWidth);
    h_robot{i}(1,2) = circles(footPrint_i(1,1),footPrint_i(1,2), 0.3,'edgecolor','black','facecolor','yellow','facealpha',1.0);
end