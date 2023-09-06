%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Zongyuan Shen %
% All Rights Reserved   %
%%%%%%%%%%%%%%%%%%%%%%%%%

function  [dynaObs_i, tree_i, path_i, footPrint_i, ...
    dynObsInfo, treeInfo, pathInfo, footPrintInfo]...
    = dataExtract(dynObsInfo, treeInfo, pathInfo, footPrintInfo, i)
    
    dynaObs_i = [];
    tree_i = [];
    path_i = [];
   
    for j = 1:height(dynObsInfo)   
        if dynObsInfo(j,1:width(dynObsInfo)) == i % Find the line with next iteration index
            if j > 2
                dynaObs_i = dynObsInfo(2:j-1,:); % read valid data
            else
                dynaObs_i = []; % no valid data
            end
            dynObsInfo(1:j-1,:) = [];
            break;
        elseif j == height(dynObsInfo) % There is only one iteration data
            if j > 1
                dynaObs_i = dynObsInfo(2:j,:); % read valid data
            else
                dynaObs_i = []; % no valid data
            end
            dynObsInfo(1:j,:) = [];
            break;
        end
    end

    for j = 1:height(treeInfo)   
        if treeInfo(j,1:width(treeInfo)) == i % Find the line with next iteration index
            if j > 2
                tree_i = treeInfo(2:j-1,:); % read valid data
            else
                tree_i = []; % no valid data
            end
            treeInfo(1:j-1,:) = [];
            break;
        elseif j == height(treeInfo) % There is only one iteration data
            if j > 1
                tree_i = treeInfo(2:j,:); % read valid data
            else
                tree_i = []; % no valid data
            end
            treeInfo(1:j,:) = [];
            break;
        end
    end

    for j = 1:height(pathInfo)   
        if pathInfo(j,1:width(pathInfo)) == i % Find the line with next iteration index
            if j > 2
                path_i = pathInfo(2:j-1,:); % read valid data
            else
                path_i = []; % no valid data
            end
            pathInfo(1:j-1,:) = [];
            break;
        elseif j == height(pathInfo) % There is only one iteration data
            if j > 1
                path_i = pathInfo(2:j,:); % read valid data
            else
                path_i = []; % no valid data
            end
            pathInfo(1:j,:) = [];
            break;
        end
    end

    footPrint_i = footPrintInfo(1,:);
    footPrintInfo(1,:) = [];
end