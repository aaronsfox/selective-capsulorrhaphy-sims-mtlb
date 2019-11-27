function dist = dist_markers2D(M1,M2)

% function dist = dist_markers(M1,M2)
%
% This function calculates the distance between two markers
% M1  is an array with markers in order of [X Y] of marker 1
% M2  is an array with markers in order of [X Y] of marker 2

dist = sqrt((M1(:,1)-M2(:,1)).^2+(M1(:,2)-M2(:,2)).^2);

