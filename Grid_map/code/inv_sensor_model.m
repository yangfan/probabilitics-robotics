function [mapUpdate, robPoseMapFrame, laserEndPntsMapFrame] = inv_sensor_model(map, scan, robPose, gridSize, offset, probOcc, probFree)
% Compute the log odds values that should be added to the map based on the inverse sensor model
% of a laser range finder.

% map is the matrix containing the occupancy values (IN LOG ODDS) of each cell in the map.
% scan is a laser scan made at this time step. Contains the range readings of each laser beam.
% robPose is the robot pose in the world coordinates frame.
% gridSize is the size of each grid in meters.
% offset = [offsetX; offsetY] is the offset that needs to be subtracted from a point
% when converting to map coordinates.
% probOcc is the probability that a cell is occupied by an obstacle given that a
% laser beam endpoint hit that cell.
% probFree is the probability that a cell is occupied given that a laser beam passed through it.

% mapUpdate is a matrix of the same size as map. It has the log odds values that need to be added for the cells
% affected by the current laser scan. All unaffected cells should be zeros.
% robPoseMapFrame is the pose of the robot in the map coordinates frame.
% laserEndPntsMapFrame are map coordinates of the endpoints of each laser beam (also used for visualization purposes).

% Initialize mapUpdate.
mapUpdate = zeros(size(map));

% Robot pose as a homogeneous transformation matrix.
robTrans = v2t(robPose);

% compute robPoseMapFrame. Use world_to_map_coordinates implementation.
robPoseMapFrame = world_to_map_coordinates(robPose, gridSize, offset);

% Compute the Cartesian coordinates of the laser beam endpoints.
% Set the third argument to 'true' to use only half the beams for speeding up the algorithm when debugging.
laserEndPnts = robotlaser_as_cartesian(scan, 30, false);

% Compute the endpoints of the laser beams in the world coordinates frame.
laserEndPnts = robTrans*laserEndPnts;
% compute laserEndPntsMapFrame from laserEndPnts. Use world_to_map_coordinates implementation.
laserEndPntsMapFrame = world_to_map_coordinates(laserEndPnts, gridSize, offset);
laserEndPntsMapFrame = unique(laserEndPntsMapFrame', 'rows')';

% freeCells are the map coordinates of the cells through which the laser beams pass.
freeCells = [];

% Iterate over each laser beam and compute freeCells.
% Use the bresenham method available to you in tools for computing the X and Y
% coordinates of the points that lie on a line.
for sc=1:columns(laserEndPntsMapFrame)
  
  % compute the XY map coordinates of the free cells along the laser beam ending in laserEndPntsMapFrame(:,sc)
  [X,Y] = bresenham([robPoseMapFrame(1), robPoseMapFrame(2); laserEndPntsMapFrame(:,sc)(1), laserEndPntsMapFrame(:,sc)(2)]);
  % add them to freeCells
  freeCells = [freeCells; [X(1:end-1)', Y(1:end-1)']];

endfor


% update the log odds values in mapUpdate for each free cell according to probFree.
idx = sub2ind(size(mapUpdate), freeCells(:,1), freeCells(:,2));
mapUpdate(idx) = prob_to_log_odds(probFree);

% update the log odds values in mapUpdate for each laser endpoint according to probOcc.
idx = sub2ind(size(mapUpdate), laserEndPntsMapFrame(1,:), laserEndPntsMapFrame(2,:));
mapUpdate(idx) = prob_to_log_odds(probOcc);

end
