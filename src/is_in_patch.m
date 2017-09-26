%	IS_IN_PATCH - Determines if a point is inside a patch
% 
% Syntax:  is_in_patch(event_x, event_y, patches(patch_id), PATCH_WIDTH)
%
% Inputs:
%   -   pt_x: x position of the point
%   -   pt_y: y position of the point
%   -   patch: patch structure as defined in "features_main.m"
%   -   PATCH_WIDTH: Width of the patch
%
% Outputs:
%    Boolean: true if the event is in the patch
%
% Example: 
%    if is_in_patch(event_x, event_y, patches(patch_id), PATCH_WIDTH)
%           ...
% 
% Author:   Thomas Lew
% email:    lewt@ethz.ch
% Website:  https://github.com/thomasjlew/
% September 2017; Last revision: 22-September-2017

%------------- BEGIN CODE --------------

function [ b_is_inside_patch ] = is_in_patch( pt_x, pt_y, patch, PATCH_WIDTH )
%IS_IN_PATCH - Returns true if the point is inside the patch
%   Inputs:
%       patch - structure containing all elements of a patch
%           patch.feat_pos   -   x&y coord of features point wrt. img
%           patch.model_pts  -   coord of model points wrt. image
%       pt_x & pt_y - point coordinates wrt. image
    if  pt_x >= (patch.feat_pos(1) - PATCH_WIDTH/2) && ...
        pt_x <= (patch.feat_pos(1) + PATCH_WIDTH/2) && ...
        pt_y >= (patch.feat_pos(2) - PATCH_WIDTH/2) && ...
        pt_y <= (patch.feat_pos(2) + PATCH_WIDTH/2)
        b_is_inside_patch = true;
        return
    else
        b_is_inside_patch = false;
        return
    end
end

%------------- END OF CODE --------------