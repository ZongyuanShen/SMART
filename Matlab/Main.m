%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Zongyuan Shen %
% All Rights Reserved   %
%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
close all;
clc;

%% Setup the video record
videoRecord = true;
if videoRecord == true
    disp('recording...')
    writerObj=VideoWriter('Demo','MPEG-4');
    writerObj.FrameRate = 10;
    open(writerObj);
end

%% Setup parameters
global LRZ_radius OHZ_radius obstacleRadius cellSize;
ROW = 32;
COL = 32;
TRH = 0.8;
TOH = 0.4;
Robot_SpeedMax = 4.0;
Obstacle_SpeedMax = 4.0;
obstacleRadius = 1.0;
LRZ_radius = TRH * Robot_SpeedMax;
OHZ_radius = TOH * Obstacle_SpeedMax + obstacleRadius;
cellSize = 1;
start = [2, 2];
goal = [30, 30];

h_obs=[];
h_tree=[];
h_path=[];
h_robot = [];

folder = 'dir_to_the_fold_of_data';

%% Load data
[dynObsInfo, treeInfo, pathInfo, footPrintInfo, staticObsMap] = dataLoad(folder);

%% Plot Environment map
envMapPlot(staticObsMap, start, goal);

%% Visualize each frame
for i = 1:height(footPrintInfo)

    % Remove previous figure
    if i > 1
        delete(h_obs{i-1});
        delete(h_tree{i-1});
        delete(h_robot{i-1});
        delete(h_path{i-1});
    end
    
    % Extract data at time i
    [dynaObs_i, tree_i, path_i, footPrint_i, ...
    dynObsInfo, treeInfo, pathInfo, footPrintInfo]...
    = dataExtract(dynObsInfo, treeInfo, pathInfo, footPrintInfo, i);

    % Visualize data
    [h_robot, h_obs, h_tree, h_path] ...
    = dataVisualize(h_robot, h_obs, h_tree, h_path,...
    dynaObs_i, tree_i, path_i, footPrint_i, i);
    
    %pause(0.001);
     
    % Record the frame at the current iteration
    if videoRecord == true
            axesValue = axis;
            axis(axesValue);
            frame = getframe;
            try
                writeVideo(writerObj,frame);
            catch
                disp('something')
            end
    end
  
    
end

if videoRecord == true
    close(writerObj);
    fprintf('Terminate video record\n');
end