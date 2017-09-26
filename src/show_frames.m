% SHOW_FRAMES - Show all frames quickly from "The Event-Camera Dataset" [1]

% Syntax:  show_frames
%
% Inputs:
%    Frames from "The Event-Camera Dataset" [2]
% 
% Outputs:
%    Images shown sequentially from the set [2]
%
% Example: 
%    show_frames
%
% Other m-files required:   none

% Subfunctions: none
% MAT-files required: none
% Dataset required: Any set from "The Event-Camera Dataset" [1]

% References:
%   [1] E. Mueggler, H. Rebecq, G. Gallego, T. Delbruck, D. Scaramuzza
%       The Event-Camera Dataset and Simulator: Event-based Data for Pose 
%       Estimation, Visual Odometry, and SLAM, International Journal of 
%       Robotics Research, Vol. 36, Issue 2, pages 142-149, Feb. 2017.

% Author:   Thomas Lew
% email:    lewt@ethz.ch
% Website:  https://github.com/thomasjlew/
% September 2017; Last revision: 26-September-2017

%------------- BEGIN CODE --------------
clc;
close all;
clear all;

DATASET_PATH = './shapes_6dof/';
% DATASET_PATH = './poster_6dof/';

B_EXTRACT_FRAMES_ONLY_ONCE = true;
NB_MAX_FRAMES = inf;
SIZE_FRAME_IMG = 0; %   Initialized in the code
% NB_STRONGEST_FEAT = 30;
NB_STRONGEST_FEAT = 4;
PATCH_WIDTH = 24; % We use square patches of width 25 (around middle pixel)

NB_MAX_EVENTS_PER_FRAME = 10000; % put 100 is max I think
B_PLOT_EVENTS = false;
B_PLOT_SUBPLOT3 = true;
PATCH_PLOT_ID = 3;

global sb2_fh;
global sb3_fh;  % necessary to erase it within a function
global sb3_axis;


% From paper:  In practice,
% it is more efficient to compute the registration transformation
% every M events, e.g., of half the size of the model point set.
NEW_EVENTS_TO_REGISTR_FACTOR = 2;


%   Open .txt files
frames_fileID = fopen(strcat(DATASET_PATH,'images.txt'),'r');
events_fileID = fopen(strcat(DATASET_PATH,'events.txt'),'r');


for frame_nb = 1:NB_MAX_FRAMES
    fprintf('---------------\n');
    fprintf(strcat('New frame nb: ',int2str(frame_nb),'\n'));
    fprintf('---------------\n');
    
    %   --------------------------------------
    %   Find, Read and Display the frame image
    %   --------------------------------------
    %   Find image from header
    tmp_frame_header=textscan(frames_fileID,'%f %s',1,'Delimiter','\n'); % OPTIMIZATION: REPLACE WITH GETL & SSCANF
    frame_cur_t = tmp_frame_header{1,1};
    frame_cur_img_file = strcat(DATASET_PATH,cell2mat(tmp_frame_header{1,2}));
    %   Read & display image
    frame_cur_img = imread(frame_cur_img_file);
    SIZE_FRAME_IMG = size(frame_cur_img);
    
%     hFig = figure(1);
%     set(hFig, 'Position', [100 1000 1000 1000])
    imshow(frame_cur_img);
%     subplot1_setup(strcat('Raw Image n°', int2str(frame_nb),' with features'), ...
%                     frame_cur_img);
end


%%  Plotting function
function subplot1_setup(my_title, image)
    hFig = figure(1);
    set(hFig, 'Position', [100 0 1000 1000])
    
    subplot(2,2,1);
    imshow(image);
    title(my_title);
    hold on;
end