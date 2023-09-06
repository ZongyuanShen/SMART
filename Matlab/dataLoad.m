%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Zongyuan Shen %
% All Rights Reserved   %
%%%%%%%%%%%%%%%%%%%%%%%%%

function [dynObsInfo, treeInfo, pathInfo, footPrintInfo, staticObsMap] = dataLoad(folder)
    dynObsInfo = load(append(folder, 'dynObsInfo.txt'));
    treeInfo = load(append(folder, 'treeInfo.txt'));
    pathInfo = load(append(folder, 'pathInfo.txt'));
    footPrintInfo = load(append(folder, 'footPrintInfo.txt'));
    staticObsMap = load(append(folder, 'staticObsMap.txt'));
end