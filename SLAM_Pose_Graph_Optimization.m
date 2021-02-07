%%The goal of this example is to build a map of the environment using the lidar scans and retrieve the trajectory of the robot.
%To build the map of the environment, the SLAM algorithm incrementally processes the lidar 
%scans and builds a pose graph that links these scans. The robot recognizes a previously-visited
%place through scan matching and may establish one or more loop closures along its moving path. 
%The SLAM algorithm utilizes the loop closure information to update the map and adjust the estimated 
%robot trajectory.


%Create a lidarSLAM object and set the map resolution and the max lidar range.
load('offlineSlamData.mat');
maxLidarRange = 8;
mapResolution = 20;
slamAlg = lidarSLAM(mapResolution, maxLidarRange);

%setting  loop closure parameters
slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 8;

%Observe the Map Building Process with Initial 10 Scans
for i=1:10
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scans{i});
    if isScanAccepted
        fprintf('Added scan %d \n', i);
    end
end

%Reconstruct the scene by plotting the scans and poses tracked by the slamAlg.
figure;
show(slamAlg);
title({'Map of the Environment','Pose Graph for Initial 10 Scans'});

firstTimeLCDetected = false;

%Continue to add scans in a loop
figure;
for i=10:length(scans)
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scans{i});
    if ~isScanAccepted
        continue;
    end
    % visualize the first detected loop closure, if you want to see the
    % complete map building process, remove the if condition below
    if optimizationInfo.IsPerformed && ~firstTimeLCDetected
        show(slamAlg, 'Poses', 'off');
        hold on;
        show(slamAlg.PoseGraph); 
        hold off;
        firstTimeLCDetected = true;
        drawnow
    end
end
title('First loop closure');


%Plot the final built map after all scans are added to the slamAlg object
figure
show(slamAlg);
title({'Final Built Map of the Environment', 'Trajectory of the Robot'});

%The optimized scans and poses can be used to generate a occupancyMap, which represents the environment as a probabilistic occupancy grid.
[scans, optimizedPoses]  = scansAndPoses(slamAlg);
map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);


%Visualize the occupancy grid map populated with the laser scans and the optimized pose graph
figure; 
show(map);
hold on
show(slamAlg.PoseGraph, 'IDs', 'off');
hold off
title('Occupancy Grid Map Built Using Lidar SLAM');