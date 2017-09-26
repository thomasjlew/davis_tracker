%	FEATURES_MAIN - Extracts features from frames (usual images) and 
%                   tracks them using events from an "Event-Camera".
%                   Implements the method described in [1]
% 
% Syntax:  features_main
%
% Inputs:
%    Frames from "The Event-Camera Dataset" [2]
%    Events from "The Event-Camera Dataset" [2]
%
% Outputs:
%    Images with tracked features, depending on the defined boolean values
%
% Example: 
%    features_main
%
% Other m-files required:   icp.m [3]
%                           knnsearch.m
%                           is_in_patch.m
% Subfunctions: none
% MAT-files required: none
% Dataset required:     Any set from "The Event-Camera Dataset" [2]

% References:
%   [1] D. Tedaldi, G. Gallego, E. Mueggler, and D. Scaramuzza, “Feature
%       detection and tracking with the dynamic and active-pixel vision 
%       sensor (DAVIS),” in Int. Conf. on Event-Based Control, Comm. and 
%       Signal Proc. (EBCCSP), Krakow, Poland, Jun. 2016.
%   [2] E. Mueggler, H. Rebecq, G. Gallego, T. Delbruck, D. Scaramuzza
%       The Event-Camera Dataset and Simulator: Event-based Data for Pose 
%       Estimation, Visual Odometry, and SLAM, International Journal of 
%       Robotics Research, Vol. 36, Issue 2, pages 142-149, Feb. 2017.
%   [3] P. Besl and N. D. McKay, “A method for registration of 3-D shapes,”
%       IEEE Trans. Pattern Anal. Machine Intell., vol. 14, no. 2, 
%       pp. 239–256, 1992.

% Author:   Thomas Lew
% email:    lewt@ethz.ch
% Website:  https://github.com/thomasjlew/
% September 2017; Last revision: 26-September-2017

%------------- BEGIN CODE --------------

%%  Definitions

%   ******************
%   PATCHES Definition
%   ******************
%   patches=features_detect(frame_cur_img)
%   Structure containing all elements: The feature takes coord. (0,0)
%   in local coordinate system
%   - patches(1).feat_pos   -   x&y coord of features point wrt. img
%   - //patches(1).binary_img -   edges in patch, 1 or 0    // NOT USED
%   - patches(1).model_pts  -   coord of model points wrt. to image
%   - patches(1).nb_new_events - nb new events after last registration
%   ******************


%   ****************
%   PLOTS Definition
%   ****************
%     sb1_fh = subplot(2,2,1) - RAW image with features + events
%     sb2_fh = subplot(2,2,2) - EDGES image with features + patches
%     sb3_fh = subplot(2,2,3) - View of patch with data points + model points
%   ****************


%% Main features stracker
clc;
close all;
clear all;

global sb2_fh;
global sb3_fh;  % necessary to erase it within a function
global sb3_axis;

% Datasets from "The Event-Camera Dataset". See 'Dataset required' above.
DATASET_PATH = './shapes_6dof/';
% DATASET_PATH = './poster_6dof/';

B_EXTRACT_FRAMES_ONLY_ONCE = true;
NB_MAX_FRAMES = inf;
SIZE_FRAME_IMG = 0; %   Initialized in the code
NB_STRONGEST_FEAT = 4;  % number of new features to be tracked
PATCH_WIDTH = 24; % We use square patches of width 25 (around middle pixel)

% NB_MAX_EVENTS_PER_FRAME = 10000; %  Should be removed
B_PLOT_EVENTS = false;
B_PLOT_SUBPLOT3 = false;
PATCH_PLOT_ID = 3;

% From paper [1]:  "In practice,
% it is more efficient to compute the registration transformation
% every M events, e.g., of half the size of the model point set."
NEW_EVENTS_TO_REGISTR_FACTOR = 2;

%   Open .txt files
frames_fileID = fopen(strcat(DATASET_PATH,'images.txt'),'r');
events_fileID = fopen(strcat(DATASET_PATH,'events.txt'),'r');

%   Initialize patches
patches = [];


%%  MAIN LOOP
%   Process NB_MAX_FRAMES frames and events until that timestamp
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
    
    subplot1_setup(strcat('Raw Image n°', int2str(frame_nb),' with features'), ...
                    frame_cur_img);
    %   --------------------------------------
    
    
    %   -------------------------------------------
    %   -------------------------------------------
    %       FEATURES DETECTION from frame image
    %   -------------------------------------------
    %   -------------------------------------------
    
    %   1) Extract Corner points on the frame (Harris detector)
    new_corners = detectHarrisFeatures(frame_cur_img);
    nb_new_corners = min(NB_STRONGEST_FEAT,new_corners.Count);
    new_corners = new_corners.selectStrongest(nb_new_corners);
    new_corners.Location = (new_corners.Location);  %   Convert to Integer
    
    %   plot new_corners (features) in green and saved patches in blue
    subplot_plot_corners_feat(1, new_corners, patches);
    
        
    %   2) Extract and plotEdges (Canny edge detector) 
    %       (returns a binary image, 1 if edge pixel; 0 otherwise).
    edges_img = edge(frame_cur_img,'canny');

    %   ------------------------------------------------------------
    %   3a) Extract local edge-map patches around corner points, and
    % convert them into patches and model point sets.
    for feat_id=1:nb_new_corners
        %   Add these corner points as new patches
        new_patch = [];
        new_patch.feat_pos = (new_corners(feat_id).Location); %    !! OPTIMIZE CODE HERE !!
        %   Initialize patch structure fields
        %new_patch.binary_img = zeros(PATCH_WIDTH+1,PATCH_WIDTH+1);
        new_patch.model_pts = [];
        new_patch.data_pts = [];
        new_patch.nb_new_events = 0;
        
        for id_x=1:PATCH_WIDTH+1
            for id_y=1:PATCH_WIDTH+1
                %   Check if patch element is out of image
                if (new_patch.feat_pos(1) - PATCH_WIDTH/2 + id_x) < 0 || ...   %   x
                   (new_patch.feat_pos(1) - PATCH_WIDTH/2 + id_x) > SIZE_FRAME_IMG(2) || ...
                   (new_patch.feat_pos(2) - PATCH_WIDTH/2 + id_y) < 0 || ...   %   y
                   (new_patch.feat_pos(2) - PATCH_WIDTH/2 + id_y) > SIZE_FRAME_IMG(1)
                    disp('OUT OF IMAGE!!!')
                    break
                end
                
                %   Add model points ( defined in patch coordinates, with 
                %                       zero at top left corner )
                if edges_img(int16(new_patch.feat_pos(2)) - PATCH_WIDTH/2 + id_y, ... 
                             int16(new_patch.feat_pos(1)) - PATCH_WIDTH/2 + id_x) 
                    %new_patch.binary_img(id_y,id_x) = 1;
                    new_model_pt = [new_patch.feat_pos(1) - PATCH_WIDTH/2 + id_x; ...
                                    new_patch.feat_pos(2) - PATCH_WIDTH/2 + id_y];
                    new_patch.model_pts = [new_patch.model_pts, single(new_model_pt)];
                end
            end
        end
            
        %   If B_EXTRACT_FRAMES_ONLY_ONCE is true, then only 1st frame
        %   features will be extracted added to patches
        if not(B_EXTRACT_FRAMES_ONLY_ONCE && (frame_nb > 1))
            disp('adding new patch')
            %   Add to new patch struct. to patches vector
            patches = [patches, new_patch];
        end
    end
    %   END  FEATURES DETECTION from frame image
    %   -------------------------------------------
    %   -------------------------------------------

    %   -------------------------------------------------------------------
    %   FEATURES DETECTION PLOTTING
    %   -------------------------------------------------------------------
    %   SUBPLOT 2: Plot features locations on the 2nd subplot
    %               blue: tracked features
    %               green: new extracted corners (features)
    subplot2_setup('Edges and Locations of patches', edges_img);
    subplot_plot_corners_feat(2, new_corners, patches);
    for feat_id=1:size(new_corners,1)%nb_new_corners
        patch_plot_contours(sb2_fh,new_corners(feat_id).Location,PATCH_WIDTH,'r'); hold on;
    end
    patch_plot_contours(sb2_fh,patches(PATCH_PLOT_ID).feat_pos,PATCH_WIDTH,'y');
    
    %   Debugging: Display one patch more closely
    %   Display the patch of interest in yellow
    %   SUBPLOT 3: Plot Model&data points of the patch
    if B_PLOT_SUBPLOT3
        subplot3_setup(strcat('Model & Data Points of patch n�',int2str(PATCH_PLOT_ID)), ...
                                             patches, PATCH_PLOT_ID, PATCH_WIDTH);
    end
    %   END FEATURES DETECTION PLOTTING
    %   -------------------------------------------------------------------
    
    
    
    
    %   -------------------------------------------------------------------
    %   -------------------------------------------------------------------
    %   	FEATURES TRACKING from events
    %   -------------------------------------------------------------------
    %   -------------------------------------------------------------------
    
    %   Process events until frame timestamp
    disp(' >> Processing events');
    event_cur_t = 0;
    while (event_cur_t < frame_cur_t)   %     for event_nb = 1:NB_MAX_EVENTS_PER_FRAME
%         disp('  >> event nb: ');disp(event_nb);
%         tmp_event=textscan(events_fileID,'%f %d %d %d',1,'Delimiter','\n'); % almost double of time!!!
%         event_cur_t = tmp_event{1,1};
        tmp_event=fgetl(events_fileID);tmp_event=sscanf(tmp_event,'%f %d %d %d');
        event_cur_t = tmp_event(1);
%             event_x = int16(tmp_event{1,2});
%             event_y = int16(tmp_event{1,3});
%             event_pol = tmp_event{1,4};
        event_x = tmp_event(2);
        event_y = tmp_event(3);
        event_pol = tmp_event(4);
        if(B_PLOT_EVENTS)
            subplot(2,2,1);
            scatter(event_x,event_y,'r');
        end

        %   Check in which patches this event belongs to and add to 
        %       model point set of the patch
        for patch_id=1:length(patches)
            if(is_in_patch(event_x, event_y, patches(patch_id), PATCH_WIDTH)==true)
                %   --------------------
                %   FEATURE REGISTRATION
                %   --------------------

                %   --------------------
                %   1) REJECT OUTLIERS
                %       opt1: define mask and multiply with patch mask
                %               todo and test speed (+memory...)
                %       opt2: compute all distances
                b_event_is_outlier = true;                      %%% !!!! -> -> -> OPTIMIZATION: INCLUDE THIS AFTER REGISTRATION WITH NEAREST NEIGHBOURS  !!!!
                for model_pt_id = 1:size(patches(patch_id).model_pts,2)
                    if abs(event_x-patches(patch_id).model_pts(1,model_pt_id)) <= 2 && ...
                        abs(event_y-patches(patch_id).model_pts(2,model_pt_id)) <= 2
%                             disp('event NOT outlier :)');
                        b_event_is_outlier = false;
                        patches(patch_id).nb_new_events = patches(patch_id).nb_new_events + 1;
                        break;
                    end
                end
                if(b_event_is_outlier)
%                         disp('EVENT is OUTLIER!');
                    break;
                end

                %   Display a debugging patch
                if (patch_id==PATCH_PLOT_ID) && B_PLOT_SUBPLOT3
%                         disp(strcat('1 new event in patch nb�',int2str(PATCH_PLOT_ID),'!'));
                    subplot(2,2,3); hold on;
                    scatter(event_x, event_y,'b');
                    pause(0.000000000001); % Otherwise, plot doesn't displays...   !!! FIX THAT !!!
                end

                %   --------------------

                %   Update the corresponding data point set
                patches(patch_id).data_pts = [patches(patch_id).data_pts, [event_x; event_y]];

                %   --------------------------------------------
                %   2) Compute the REGISTRATION TRANSFORMATION A 
                %   between the matched point sets using the iterative 
                %   closest point algorithm
                %   --------------------------------------------
                if patches(patch_id).nb_new_events > ...
                   size(patches(patch_id).model_pts,2) / NEW_EVENTS_TO_REGISTR_FACTOR

%                         disp(strcat('Computing registration for patch nb�',int2str(PATCH_PLOT_ID)));
                    patches(patch_id).nb_new_events = 0;

                    %   Get neirest neighbours and convert to 3D point cloud
%                         [ID_model, dists] = knnsearch(single(patches(patch_id).model_pts)',single(patches(patch_id).data_pts)'); % matlab knn func
                    [ID_model, dists] = knnsearch(single(patches(patch_id).data_pts)',single(patches(patch_id).model_pts)'); % knnsearch added func
                    AVG_dists = sum(dists)/size(patches(patch_id).data_pts,2);

                    %       REJECT OUTLIERS  %%% !!!! -> -> -> OPTIMIZATION: INCLUDE THIS AFTER REGISTRATION WITH NEAREST NEIGHBOURS  !!!!



                    cloud_model_pts = zeros(3,size(patches(patch_id).data_pts,2)); %    after outliers rejection: change the size definition here
                    cloud_data_pts = zeros(3,size(patches(patch_id).data_pts,2));
                    for id = 1:size(cloud_model_pts,2)
                        if (patch_id == PATCH_PLOT_ID) && B_PLOT_SUBPLOT3
                            scatter(single(patches(patch_id).model_pts(1,ID_model(id))),single(patches(patch_id).model_pts(2,ID_model(id))),'c*');
                            scatter(single(patches(patch_id).data_pts(1,id)),single(patches(patch_id).data_pts(2,id)),'g');
                            pause(0.00001); % Otherwise, plot doesn't displays...   !!! FIX THAT !!!
                        end

                        cloud_model_pts(1:3,id) = [single(patches(patch_id).model_pts(1:2,ID_model(id)));1];
                        cloud_data_pts(1:3,id) = [single(patches(patch_id).data_pts(1:2,id));1];
                    end
                    pt_cloud_model_pts = pointCloud(cloud_model_pts');
                    pt_cloud_data_pts = pointCloud(cloud_data_pts');
%                         pt_cloud_model_pts = pointCloud([(single(patches(patch_id).model_pts))',ones(size(patches(patch_id).model_pts,2),1)]);
%                         pt_cloud_data_pts = pointCloud([(single(patches(patch_id).data_pts))',ones(size(patches(patch_id).data_pts,2),1)]);

                    %   Compute 3D Euclidean transformation         !!! OPTIMISATION: LATEST PAPER USES WEIGHTS TO MAKE IT MOREPRECISE !!!
%                         A = pcregrigid(pt_cloud_model_pts,pt_cloud_data_pts) ;
                    [R, t, ER] = icp(pt_cloud_model_pts.Location',pt_cloud_data_pts.Location');
                    t=-t(1:2)';
                    R=R(1:2,1:2);
%                         A = pcregrigid(pt_cloud_data_pts,pt_cloud_model_pts) ;

%                         pt_cloud_model_pts = pctransform(pt_cloud_model_pts,A);
%                         for i=1:size(patches(patch_id).model_pts)
%                             patches(patch_id).model_pts(1) = pt_cloud_model_pts.Location(i,1);
%                             patches(patch_id).model_pts(2) = pt_cloud_model_pts.Location(i,2);
%                         end
%                         pt_cld_feat = pointCloud([single(patches(patch_id).feat_pos(1)),single(patches(patch_id).feat_pos(2)),1]);
%                         pt_cld_feat = pctransform(pt_cld_feat,A);
%                         patches(patch_id).feat_pos(1) = pt_cld_feat.Location(1,1);
%                         patches(patch_id).feat_pos(2) = pt_cld_feat.Location(1,2);


                    %   Extract 2D Euclidean transformation
%                         R = A.T(1:2,1:2);
%                         t = A.T(4,1:2);
                    %   --------------------------------------------

                    %   -----------------------------------------------
                    %   3) Update registration parameters of the 
                    %       feature and model points
                    %   -----------------------------------------------
%                         if patch_id == PATCH_PLOT_ID
%                             R
%                             t
%                             patches(patch_id).feat_pos
%                         end
                    patches(patch_id).feat_pos = (R' * single(patches(patch_id).feat_pos') + t')';
                    patches(patch_id).model_pts = (R' * single(patches(patch_id).model_pts) + t');
%                         if patch_id == PATCH_PLOT_ID
%                             patches(patch_id).feat_pos
%                         end
                    %   -----------------------------------------------

                    %   -----------------------------------------------
                    %   Remove older data points
%                         patches(patch_id).data_pts = [];
                    patches(patch_id).data_pts = patches(patch_id).data_pts(1:2,uint16(size(patches(patch_id).data_pts,2)/NEW_EVENTS_TO_REGISTR_FACTOR):end);
                    %   -----------------------------------------------

                    %   -------------------------
                    %   Plot new model points set
                    %   -------------------------
                    if (patch_id == PATCH_PLOT_ID) && B_PLOT_SUBPLOT3
                        subplot3_setup(strcat('Model & Data Points of patch n.',int2str(PATCH_PLOT_ID)), ...
                                         patches, PATCH_PLOT_ID, PATCH_WIDTH);
%                             pause;
                    end
                    %   -------------------------
                end
            end
        end
    end
    %   END FEATURES TRACKING from events
    %   ---------------------------------------
    %   ---------------------------------------
    
    %   --------------------------
    %   Redraw all data points set (otherwise only new events drawn)
    if B_PLOT_SUBPLOT3
        sb3_fh = subplot(2,2,3); hold on;
        scatter(patches(PATCH_PLOT_ID).data_pts(1,1:end),patches(PATCH_PLOT_ID).data_pts(2,1:end),'MarkerEdgeColor','b');%[255/255,165.0/255,0.0]);
        set(gca,'Ydir','reverse'); % Image y axis going downwards
        plot(patches(PATCH_PLOT_ID).feat_pos(1),patches(PATCH_PLOT_ID).feat_pos(2),'b+','MarkerSize',20,'LineWidth',3);
        axis(sb3_fh,sb3_axis);
        title(strcat('Model Points of patch n.',int2str(PATCH_PLOT_ID)));
    end
    %     pause
end

fclose('all');
%------------- END OF MAIN FUNCTION --------------



%%  Plotting functions
function subplot1_setup(my_title, image)
    hFig = figure(1);
    set(hFig, 'Position', [100 0 1000 1000])
    
    subplot(2,2,1);
    imshow(image);
    title(my_title);
    hold on;
end

function subplot_plot_corners_feat(subplot_id, new_corners, patches)
    subplot(2,2,subplot_id);
    
    plot(new_corners);      %    in green
    if min(size(patches)) > 0
        for i=1:size(patches,2)
            plot(patches(i).feat_pos(1),patches(i).feat_pos(2),'b+');
        end
    end
end

function subplot2_setup(my_title, image)
    global sb2_fh;
    
    sb2_fh = subplot(2,2,2);
    imshow(image); 
    title(my_title);
    hold on;     
end

function subplot3_setup(my_title, patches, PATCH_PLOT_ID, PATCH_WIDTH)
    global sb3_fh;
    global sb3_axis;
    
    if exist('sb3_fh','var')    delete(sb3_fh);     end
    sb3_fh = subplot(2,2,3); hold on;
    sb3_axis=[  patches(PATCH_PLOT_ID).feat_pos(1)-PATCH_WIDTH/2 - 3, 
                patches(PATCH_PLOT_ID).feat_pos(1)+PATCH_WIDTH/2 + 3, 
                patches(PATCH_PLOT_ID).feat_pos(2)-PATCH_WIDTH/2 - 3, 
                patches(PATCH_PLOT_ID).feat_pos(2)+PATCH_WIDTH/2 + 3    ];
    axis(sb3_fh,sb3_axis);
    
    %   Model points
    scatter(patches(PATCH_PLOT_ID).model_pts(1,1:end),patches(PATCH_PLOT_ID).model_pts(2,1:end),'r*');
    
    %   Data points
    if min(size(patches(PATCH_PLOT_ID).data_pts) > 0)
        scatter(patches(PATCH_PLOT_ID).data_pts(1,1:end),patches(PATCH_PLOT_ID).data_pts(2,1:end),'b');
    end
    
    %   Feature position
    plot(patches(PATCH_PLOT_ID).feat_pos(1),patches(PATCH_PLOT_ID).feat_pos(2),'b+','MarkerSize',20,'LineWidth',3);
    
    set(gca,'Ydir','reverse'); % Image y axis going downwards
    title(my_title);
end

function patch_plot_contours( fh, pt, SQUARE_WIDTH, clr )
%PATCH_PLOT_CONTOURS - plots a square around a feature
    plot(fh,[pt(1)-SQUARE_WIDTH/2,pt(1)+SQUARE_WIDTH/2],[pt(2)-SQUARE_WIDTH/2,pt(2)-SQUARE_WIDTH/2],clr)
    plot(fh,[pt(1)-SQUARE_WIDTH/2,pt(1)+SQUARE_WIDTH/2],[pt(2)+SQUARE_WIDTH/2,pt(2)+SQUARE_WIDTH/2],clr)
    plot(fh,[pt(1)-SQUARE_WIDTH/2,pt(1)-SQUARE_WIDTH/2],[pt(2)-SQUARE_WIDTH/2,pt(2)+SQUARE_WIDTH/2],clr)
    plot(fh,[pt(1)+SQUARE_WIDTH/2,pt(1)+SQUARE_WIDTH/2],[pt(2)-SQUARE_WIDTH/2,pt(2)+SQUARE_WIDTH/2],clr)
    
end

%------------- END OF CODE --------------