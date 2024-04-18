
data = load("routePlan.mat");

mapLayers = loadParkingLotMapLayers;
plotMapLayers(mapLayers)

costmap = combineMapLayers(mapLayers);

figure
plot(costmap, Inflation="off");
legend off

costmap.MapExtent % [x, width, y, height] in meters

costmap.CellSize  % cell size in meters

vehicleDims      = vehicleDimensions;
maxSteeringAngle = 35; % in degrees

costmap.CollisionChecker.VehicleDimensions = vehicleDims;

currentPose = [4 5 0]; % [x, y, theta]S
routePlan = data.routePlan %#ok<NOPTS>

hold on
helperPlotVehicle(currentPose, vehicleDims, DisplayName="Current Pose")
legend(Location="northwest")

for n = 1 : height(routePlan)
    vehiclePose = routePlan{n, "EndPose"};
    
    legendEntry = "Goal " + n;
    helperPlotVehicle(vehiclePose, vehicleDims, DisplayName=legendEntry);
end
hold off

behavioralPlanner = HelperBehavioralPlanner(routePlan, maxSteeringAngle);


ss = stateSpaceSE2;
ss.StateBounds = [costmap.MapExtent(1, 1:2); costmap.MapExtent(1, 3:4); -pi, pi];


validator = validatorVehicleCostmap(ss, Map=costmap);



minTurningRadius = 10; % in meters
motionPlanner = plannerHybridAStar(validator, MinTurningRadius=minTurningRadius, ...
    MotionPrimitiveLength=4); % length in meters


goalPose = routePlan{1, "EndPose"};
currentPoseRad = [currentPose(1:2) deg2rad(currentPose(3))];
goalPoseRad = [goalPose(1:2) deg2rad(goalPose(3))];
refPath = plan(motionPlanner, currentPoseRad, goalPoseRad);

plannerAxes = show(motionPlanner, Tree="off", HeadingLength=0); % Visualize the planned path.

weights = struct(Time=10, Smoothness=100, Obstacle=50);
vehicleInfo = struct("Dimension", [vehicleDims.Length, vehicleDims.Width], "Shape", "Rectangle");

localPlanner = controllerTEB(zeros(3,3),...
    MaxVelocity=[5 0.5],... % in m/s and rad/s
    MaxAcceleration=[2 0.5],... % in m/s/s and rad/s/s
    LookAheadTime=3,... % in seconds
    MinTurningRadius=minTurningRadius, NumIteration=2,...
    CostWeights=weights, RobotInformation=vehicleInfo);

maxLocalPlanDistance = 15; % in meters
localPlanner.Map = getLocalMap(costmap, currentPose, maxLocalPlanDistance);
localPlanner.ReferencePath = refPath;

[refVel, ~, localPath, ~] = localPlanner(currentPoseRad, [0 0]);

hold(plannerAxes,"on");
plot(plannerAxes, localPath(:,1), localPath(:,2), "g", LineWidth=2, DisplayName="Local Path");
legend(plannerAxes, "show", findobj(plannerAxes, Type="Line"), {"Local Path","ReferencePath"});


closeFigures; % Close all

vehicleSim = HelperVehicleSimulator(costmap, vehicleDims);

hideFigure(vehicleSim); % Hide vehicle simulation figure

vehicleSim.showTrajectory(true);
vehicleSim.showLegend(true);

vehicleSim.setVehiclePose(currentPose);
currentVel = 0;
vehicleSim.setVehicleVelocity(currentVel);

pathAnalyzer = HelperPathAnalyzer(Wheelbase=vehicleDims.Wheelbase);

sampleTime = 0.05;
lonController = HelperLongitudinalController(SampleTime=sampleTime);

controlRate = HelperFixedRate(1/sampleTime); % in Hertz

currentPose = [4 5 0]; % [x, y, theta]
vehicleSim.setVehiclePose(currentPose);

currentVel  = 0; % meters/second
vehicleSim.setVehicleVelocity(currentVel);

refPath = [];
localPath = [];

localPlanningFrequency = 1; % 1/seconds
pathAnalyzer.PlanningPeriod = 1/localPlanningFrequency/sampleTime; % timesteps

isGoalReached = false;

localPlanCount = 0; % Used for visualization only

showFigure(vehicleSim); % Show vehicle simulation figure.

while ~isGoalReached
    
    if planNextSegment(behavioralPlanner, currentPose, 2*maxLocalPlanDistance)
        [nextGoal, plannerConfig, speedConfig] = requestManeuver(behavioralPlanner, ...
            currentPose, currentVel);
        
        if isempty(refPath)
            nextStartRad = [currentPose(1:2) deg2rad(currentPose(3))];
        else
            nextStartRad = refPath(end,:);
        end
        nextGoalRad = [nextGoal(1:2) deg2rad(nextGoal(3))];
        newPath = plan(motionPlanner, nextStartRad, nextGoalRad, SearchMode="exhaustive");
        isReplanNeeded = ~checkPathValidity(newPath.States, costmap);
        if isReplanNeeded
            warning("Unable to find a valid path. Attempting to re-plan.")
            
            replanNeeded(behavioralPlanner);
        else
            refPath = [refPath; newPath.States];
            hasNewPath = true;

            vehicleSim.plotReferencePath(refPath); % Plot reference path
        end
    end

    if pathUpdateNeeded(pathAnalyzer)
        currentPose  = getVehiclePose(vehicleSim);
        currentPoseRad = [currentPose(1:2) deg2rad(currentPose(3))];
        currentVel   = getVehicleVelocity(vehicleSim);
        currentAngVel = getVehicleAngularVelocity(vehicleSim);

        localPlanner.Map = getLocalMap(costmap, currentPose, maxLocalPlanDistance);

        if hasNewPath
            localPlanner.ReferencePath = refPath;
            hasNewPath = false;
        end

        [localVel, ~, localPath, ~] = localPlanner(currentPoseRad, [currentVel currentAngVel]);

        vehicleSim.plotLocalPath(localPath); % Plot new local path

        if mod(localPlanCount, 20) == 0
            snapnow; % Capture state of the figures
        end
        localPlanCount = localPlanCount+1;

        pathAnalyzer.RefPoses     = [localPath(:,1:2) rad2deg(localPath(:,3))];
        pathAnalyzer.Directions   = sign(localVel(:,1));
        pathAnalyzer.VelocityProfile = localVel(:,1);
    end
   
    [refPose, refVel, direction] = pathAnalyzer(currentPose, currentVel);
    
    updateDrivingDirection(vehicleSim, direction);
    
    steeringAngle = lateralControllerStanley(refPose, currentPose, currentVel, ...
        Direction=direction, Wheelbase=vehicleDims.Wheelbase, PositionGain=4);
    
    lonController.Direction = direction;
    [accelCmd, decelCmd] = lonController(refVel, currentVel);
    
    drive(vehicleSim, accelCmd, decelCmd, steeringAngle);

    currentPose  = getVehiclePose(vehicleSim);
    currentVel   = getVehicleVelocity(vehicleSim);
    
    isGoalReached = helperGoalChecker(nextGoal, currentPose, currentVel, speedConfig.EndSpeed, direction);
    
    waitfor(controlRate);
end

hideFigure(vehicleSim); % Hide vehicle simulation figure

ccConfig = costmap.CollisionChecker;

figure
plot(ccConfig)
title("Current Collision Checker")

ccConfig.NumCircles = 4;

figure
plot(ccConfig)
title("New Collision Checker")

costmap.CollisionChecker = ccConfig;

figure
plot(costmap)
title("Costmap with updated collision checker")

parkMotionPlanner = pathPlannerRRT(costmap, MinIterations=1000);

 preParkPose = currentPose;

 refPath = plan(parkMotionPlanner, preParkPose, parkPose);

 figure
 plotParkingManeuver(costmap, refPath, preParkPose, parkPose)

 pause(3);
 closeFigures;
 showFigure(vehicleSim)
    
 [transitionPoses, directions] = interpolate(refPath);
    
 approxSeparation = 0.1; % meters
    
 numSmoothPoses   = round(refPath.Length / approxSeparation);
 [refPoses, directions, cumLengths, curvatures] = smoothPathSpline(transitionPoses, directions, numSmoothPoses);
  
 refVelocities = helperGenerateVelocityProfile(directions, cumLengths, curvatures, currentVel, 0, 2.2352);
       
 pathAnalyzer.RefPoses     = refPoses;
 pathAnalyzer.Directions   = directions;
 pathAnalyzer.VelocityProfile = refVelocities;
    
 vehicleSim.clearReferencePath();
 vehicleSim.clearLocalPath();
    
 reset(lonController);
    
 isGoalReached = false;
    
    while ~isGoalReached
        [refPose, refVel, direction] = pathAnalyzer(currentPose, currentVel);
    
        updateDrivingDirection(vehicleSim, direction);
    
        steeringAngle = lateralControllerStanley(refPose, currentPose, currentVel, ...
            Direction=direction, Wheelbase=vehicleDims.Wheelbase);
    
        lonController.Direction = direction;
        [accelCmd, decelCmd] = lonController(refVel, currentVel);
    
        drive(vehicleSim, accelCmd, decelCmd, steeringAngle);
    
        isGoalReached = helperGoalChecker(parkPose, currentPose, currentVel, 0, direction);
    
        waitfor(controlRate);
    
        currentPose  = getVehiclePose(vehicleSim);
        currentVel   = getVehicleVelocity(vehicleSim);
    end    
 
closeFigures;
snapnow; % Capture state of the simulation figure

delete(vehicleSim); % Delete the simulator.

function mapLayers = loadParkingLotMapLayers()

mapLayers.StationaryObstacles = imread("stationary.bmp");
mapLayers.RoadMarkings        = imread("road_markings.bmp");
mapLayers.ParkedCars          = imread("parked_cars.bmp");
end

function plotMapLayers(mapLayers)

figure
cellOfMaps = cellfun(@imcomplement, struct2cell(mapLayers), UniformOutput=false);
montage(cellOfMaps, Size=[1 numel(cellOfMaps)], Border=[5 5], ThumbnailSize=[300 NaN])
title("Map Layers - Stationary Obstacles, Road markings, and Parked Cars")
end

function costmap = combineMapLayers(mapLayers)

combinedMap = mapLayers.StationaryObstacles + mapLayers.RoadMarkings + ...
    mapLayers.ParkedCars;
combinedMap = im2single(combinedMap);

costmap = vehicleCostmap(combinedMap, CellSize=0.5);
end

function localMap = getLocalMap(costmap, pose, maxDistance)

mapExtent = costmap.MapExtent;
cellSize = costmap.CellSize;
xMin = max(pose(1)-maxDistance, mapExtent(1));
xMax = min(pose(1)+maxDistance, mapExtent(2));
yMin = max(pose(2)-maxDistance, mapExtent(3));
yMax = min(pose(2)+maxDistance, mapExtent(4));

xMinCell = round(xMin/cellSize)+1;
xMaxCell = round(xMax/cellSize);
yMaxCell = costmap.MapSize(1) - round(yMin/cellSize);
yMinCell = costmap.MapSize(1) - round(yMax/cellSize)+1;

costs = getCosts(costmap);
localCosts = costs(yMinCell:yMaxCell, xMinCell:xMaxCell);
localMap = occupancyMap(localCosts, 1/cellSize);
localMap.GridLocationInWorld = [xMin yMin];
end

function plotVelocityProfile(path, refVelocities, maxSpeed)

cumPathLength = pdist(path(:, 1:2));

plot(cumPathLength, refVelocities, LineWidth=2);

hold on
line([0;cumPathLength(end)], [maxSpeed;maxSpeed], Color="r")
hold off

buffer = 2;
xlim([0 cumPathLength(end)]);
ylim([0 maxSpeed + buffer])

xlabel("Cumulative Path Length (m)");
ylabel("Velocity (m/s)");

legend("Velocity Profile", "Max Speed")
title("Generated velocity profile")
end


function closeFigures()

figHandles = findobj(Type="figure");
for i = 1: length(figHandles)
    if ~strcmp(figHandles(i).Name, "Automated Valet Parking")
        close(figHandles(i));
    end
end
end

function plotParkingManeuver(costmap, refPath, currentPose, parkPose)

plot(costmap, Inflation="off")

hold on
plot(refPath, DisplayName="Parking Maneuver")

title("Parking Maneuver")

lo = min([currentPose(1:2); parkPose(1:2)]);
hi = max([currentPose(1:2); parkPose(1:2)]);

buffer = 6; % meters

xlim([lo(1)-buffer hi(1)+buffer])
ylim([lo(2)-buffer hi(2)+buffer])
end








































































