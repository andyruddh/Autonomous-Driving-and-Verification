%% Initialize

clear; close all; clc;

%% Simulink Model

mdl = 'ACCModel';
load_system(mdl);
set_param(mdl, 'StopTime', '1', 'FixedStep', '1');

%% Initialize variables

% Gain for distance error
K_x = 1;

% Gain for host velocity and driver-velocity error
K_v = -1;

% Gain for relative velocity between host car and lear car
K_rv = 5;

% Safe distance (in meters)
safeDist = 15;

% Vehicle length is 4.7m by default, so adjust safe distance w.r.t. ego car
vehicleLength = 4.7;
dSafe = safeDist + vehicleLength;

% Driver-set velocity (in m/s)
driverVel = 20;

% Initial positions of ego and lead (in m)
x0_ego = 10;
x0_lead = 50;

% Initial velocities of ego and lead (in m/s)
egoVel = 10;
leadVel = 20;

% Min ego and lead car velocity (in m/s)
egoMinVel = 0;
leadMinVel = 5;

% Max ego and lead car velocity (in m/s)
egoMaxVel = 40;
leadMaxVel = 5;

%% Create array data structure to store the ego and lead velocities and their relative distances

egoParams = [];
leadParams = [];
relative_distance = [];

%% Generate driving scenario

sc = drivingScenario;

egoCar = vehicle(sc,'Position',[x0_ego 0 0],'Yaw',0);
leadCar = vehicle(sc,'Position',[x0_lead 0 0],'Yaw',0);
roadLength = 1e4; % in meters
roadCenters = [0 0; roadLength/4 0; roadLength/2 0; 3*roadLength/4 0; roadLength 0];
roadWidth = 25; % in meters
road(sc, roadCenters, roadWidth);

%% Sensors

sensors = cell(8,1);
% Front-facing long-range radar sensor at the center of the front bumper of the car.
sensors{1} = radarDetectionGenerator('SensorIndex', 1, 'Height', 0.2, 'MaxRange', 174, ...
    'SensorLocation', [egoCar.Wheelbase + egoCar.FrontOverhang, 0], 'FieldOfView', [20, 5]);

% Rear-facing long-range radar sensor at the center of the rear bumper of the car.
sensors{2} = radarDetectionGenerator('SensorIndex', 2, 'Height', 0.2, 'Yaw', 180, ...
    'SensorLocation', [-egoCar.RearOverhang, 0], 'MaxRange', 174, 'FieldOfView', [20, 5]);

% Rear-left-facing short-range radar sensor at the left rear wheel well of the car.
sensors{3} = radarDetectionGenerator('SensorIndex', 3, 'Height', 0.2, 'Yaw', 120, ...
    'SensorLocation', [0, egoCar.Width/2], 'MaxRange', 30, 'ReferenceRange', 50, ...
    'FieldOfView', [90, 5], 'AzimuthResolution', 10, 'RangeResolution', 1.25);

% Rear-right-facing short-range radar sensor at the right rear wheel well of the car.
sensors{4} = radarDetectionGenerator('SensorIndex', 4, 'Height', 0.2, 'Yaw', -120, ...
    'SensorLocation', [0, -egoCar.Width/2], 'MaxRange', 30, 'ReferenceRange', 50, ...
    'FieldOfView', [90, 5], 'AzimuthResolution', 10, 'RangeResolution', 1.25);

% Front-left-facing short-range radar sensor at the left front wheel well of the car.
sensors{5} = radarDetectionGenerator('SensorIndex', 5, 'Height', 0.2, 'Yaw', 60, ...
    'SensorLocation', [egoCar.Wheelbase, egoCar.Width/2], 'MaxRange', 30, ...
    'ReferenceRange', 50, 'FieldOfView', [90, 5], 'AzimuthResolution', 10, ...
    'RangeResolution', 1.25);

% Front-right-facing short-range radar sensor at the right front wheel well of the car.
sensors{6} = radarDetectionGenerator('SensorIndex', 6, 'Height', 0.2, 'Yaw', -60, ...
    'SensorLocation', [egoCar.Wheelbase, -egoCar.Width/2], 'MaxRange', 30, ...
    'ReferenceRange', 50, 'FieldOfView', [90, 5], 'AzimuthResolution', 10, ...
    'RangeResolution', 1.25);

% Front-facing camera located at front windshield.
sensors{7} = visionDetectionGenerator('SensorIndex', 7, 'FalsePositivesPerImage', 0.1, ...
    'SensorLocation', [0.75*egoCar.Wheelbase 0], 'Height', 1.1);

% Rear-facing camera located at rear windshield.
sensors{8} = visionDetectionGenerator('SensorIndex', 8, 'FalsePositivesPerImage', 0.1, ...
    'SensorLocation', [0.2*egoCar.Wheelbase 0], 'Height', 1.1, 'Yaw', 180);

tracker = multiObjectTracker('FilterInitializationFcn', @initSimDemoFilter, ...
    'AssignmentThreshold', 30, 'ConfirmationParameters', [4 5]);
positionSelector = [1 0 0 0; 0 0 1 0]; % Position selector
velocitySelector = [0 1 0 0; 0 0 0 1]; % Velocity selector

% Create the display and return a handle to the bird's-eye plot
BEP = createDemoDisplay(egoCar, sensors);

%% Optional Plotting

chasePlot(egoCar,'Centerline','on');
set(gcf, 'Name', 'Chase Plot');
plot(sc,'RoadCenters','on','Centerline','on');
ylim([-10 10]);

%% Driving simulation

sc.SampleTime = 0.01; % Can use 0.1, 0.01, etc
sc.StopTime = 100; % Can be set to any finite time
dt = sc.SampleTime;
timeFrame = dt:dt:sc.StopTime;

% Accumulated or total distance covered by ego car
S = x0_ego;

% Accumulated or total distance covered by lead car
leadS = x0_lead;

% Flags for changing lead car velocity dynamically during simulation
leadFlag = false;
leadVel_prev = leadVel;

% Run simulation
while advance(sc) && ishghandle(BEP.Parent)
    leadCar.Position = [leadS 0 0];
    
    % Increase lead velocity to max and decrease to min
    if leadFlag
        leadVel = leadVel_prev + 1;
    elseif ~leadFlag
        leadVel = leadVel_prev - 1;
    end
    
    % Make sure car does not go backwards or exceed max limit
    if leadVel < leadMinVel
        leadVel = leadMinVel;
        leadFlag = true;
    elseif leadVel > leadMaxVel
        leadVel = leadMaxVel;
        leadFlag = false;
    end
    leadCar.Velocity = [leadVel 0 0];
    leadVel_prev = leadVel;
    
    leadS = leadS + leadVel * dt;
    leadPoses = targetPoses(egoCar);
    dRelative = leadPoses.Position(1);
    
    % Run the ACC model and get acceleration output
    sim(mdl);
    egoAcc = acc.Data(1);
    egoVel = egoVel + egoAcc * dt;
    
    % Make sure car does not go backwards or exceed max limit
    if egoVel < egoMinVel
        egoVel = egoMinVel;
    elseif egoVel > egoMaxVel
        egoVel = egoMaxVel;
    end
    
    disp('Relative Distance = ');
    disp(dRelative);
    relative_distance = [relative_distance, dRelative];
    disp('Lead Velocity = ');
    disp(leadPoses.Velocity(1));
    leadParams = [leadParams, leadVel];
    disp('Ego Velocity = ');
    disp(egoVel);
    egoParams = [egoParams, egoVel];
    
    S = S + egoVel * dt;
    egoCar.Position = [S 0 0];
    updatePlots(sc);
    drawnow;
    pause(0.001);
    
    % Get the scenario time
    time = sc.SimulationTime;
    
    % Get the position of the other vehicle in ego vehicle coordinates
    ta = targetPoses(egoCar);
    
    % Simulate the sensors
    detections = {};
    isValidTime = false(1,8);
    for i = 1:8
        [sensorDets,numValidDets,isValidTime(i)] = sensors{i}(ta, time);
        if numValidDets
            for j = 1:numValidDets
                % Vision detections do not report SNR. The tracker requires
                % that they have the same object attributes as the radar
                % detections. This adds the SNR object attribute to vision
                % detections and sets it to a NaN.
                if ~isfield(sensorDets{j}.ObjectAttributes{1}, 'SNR')
                    sensorDets{j}.ObjectAttributes{1}.SNR = NaN;
                end
            end
            detections = [detections; sensorDets]; %#ok<AGROW>
        end
    end
    
    % Update the tracker if there are new detections
    if any(isValidTime)
        vehicleLength = sensors{1}.ActorProfiles.Length;
        detectionClusters = clusterDetections(detections, vehicleLength);
        confirmedTracks = updateTracks(tracker, detectionClusters, time);
        
        % Update bird's-eye plot
        updateBEP(BEP, egoCar, detections, confirmedTracks, positionSelector, velocitySelector);
    end
end

figure;
subplot(2, 1, 1);
plot(timeFrame, egoParams, timeFrame, leadParams);
legend('EgoVel', 'LeadVel');
title('Velocities of Cars');
xlabel('Time');
ylabel('Velocity (m/s)');
subplot(2, 1, 2);
plot(timeFrame, relative_distance, 'DisplayName', 'Relative Distances');
title('Relative Distance between Ego and Lead Cars');
xlabel('Time');
ylabel('Distance (m)');

%% TODO: Robustness Check

%% Initialize a constant velocity filter based on a detection

function filter = initSimDemoFilter(detection)
% Use a 2-D constant velocity model to initialize a trackingKF filter.
% The state vector is [x;vx;y;vy]
% The detection measurement vector is [x;y;vx;vy]
% As a result, the measurement model is H = [1 0 0 0; 0 0 1 0; 0 1 0 0; 0 0 0 1]
H = [1 0 0 0; 0 0 1 0; 0 1 0 0; 0 0 0 1];
filter = trackingKF('MotionModel', '2D Constant Velocity', ...
    'State', H' * detection.Measurement, ...
    'MeasurementModel', H, ...
    'StateCovariance', H' * detection.MeasurementNoise * H, ...
    'MeasurementNoise', detection.MeasurementNoise);
end

%% Merge multiple detections suspected to be of the same vehicle to a single detection

function detectionClusters = clusterDetections(detections, vehicleSize)
N = numel(detections);
distances = zeros(N);
for i = 1:N
    for j = i+1:N
        if detections{i}.SensorIndex == detections{j}.SensorIndex
            distances(i,j) = norm(detections{i}.Measurement(1:2) - detections{j}.Measurement(1:2));
        else
            distances(i,j) = inf;
        end
    end
end
leftToCheck = 1:N;
i = 0;
detectionClusters = cell(N,1);
while ~isempty(leftToCheck)
    % Remove the detections that are in the same cluster as the one under
    % consideration
    underConsideration = leftToCheck(1);
    clusterInds = (distances(underConsideration, leftToCheck) < vehicleSize);
    detInds = leftToCheck(clusterInds);
    clusterDets = [detections{detInds}];
    clusterMeas = [clusterDets.Measurement];
    meas = mean(clusterMeas, 2);
    meas2D = [meas(1:2);meas(4:5)];
    i = i + 1;
    detectionClusters{i} = detections{detInds(1)};
    detectionClusters{i}.Measurement = meas2D;
    leftToCheck(clusterInds) = [];
end
detectionClusters(i+1:end) = [];

% Since the detections are now for clusters, modify the noise to represent
% that they are of the whole car
for i = 1:numel(detectionClusters)
    measNoise(1:2,1:2) = vehicleSize^2 * eye(2);
    measNoise(3:4,3:4) = eye(2) * 100 * vehicleSize^2;
    detectionClusters{i}.MeasurementNoise = measNoise;
end
end

%% Creates plots

function BEP = createDemoDisplay(egoCar, sensors)
% Make a figure
hFigure = figure('Position', [0, 0, 1200, 640], 'Name', 'Sensor Fusion with Synthetic Data Example');
movegui(hFigure, [0 -1]); % Moves the figure to the left and a little down from the top

% Add a car plot that follows the ego vehicle from behind
hCarViewPanel = uipanel(hFigure, 'Position', [0 0 0.5 0.5], 'Title', 'Chase Camera View');
hCarPlot = axes(hCarViewPanel);
chasePlot(egoCar, 'Centerline', 'on', 'Parent', hCarPlot);

% Add a car plot that follows the ego vehicle from a top view
hTopViewPanel = uipanel(hFigure, 'Position', [0 0.5 0.5 0.5], 'Title', 'Top View');
hCarPlot = axes(hTopViewPanel);
chasePlot(egoCar, 'Centerline', 'on', 'Parent', hCarPlot, 'ViewHeight', 130, 'ViewLocation', [0 0], 'ViewPitch', 90);

% Add a panel for a bird's-eye plot
hBEVPanel = uipanel(hFigure, 'Position', [0.5 0 0.5 1], 'Title', 'Bird''s-Eye Plot');

% Create bird's-eye plot for the ego car and sensor coverage
hBEVPlot = axes(hBEVPanel);
frontBackLim = 60;
BEP = birdsEyePlot('Parent', hBEVPlot, 'Xlimits', [-frontBackLim frontBackLim], 'Ylimits', [-35 35]);

% Plot the coverage areas for radars
for i = 1:6
    cap = coverageAreaPlotter(BEP,'FaceColor','red','EdgeColor','red');
    plotCoverageArea(cap, sensors{i}.SensorLocation,...
        sensors{i}.MaxRange, sensors{i}.Yaw, sensors{i}.FieldOfView(1));
end

% Plot the coverage areas for vision sensors
for i = 7:8
    cap = coverageAreaPlotter(BEP,'FaceColor','blue','EdgeColor','blue');
    plotCoverageArea(cap, sensors{i}.SensorLocation,...
        sensors{i}.MaxRange, sensors{i}.Yaw, 45);
end

% Create a vision detection plotter put it in a struct for future use
detectionPlotter(BEP, 'DisplayName','vision', 'MarkerEdgeColor','blue', 'Marker','^');

% Combine all radar detctions into one entry and store it for later update
detectionPlotter(BEP, 'DisplayName','radar', 'MarkerEdgeColor','red');

% Add road borders to plot
laneBoundaryPlotter(BEP, 'DisplayName','road', 'Color', [.75 .75 0]);

% Add the tracks to the bird's-eye plot. Show last 10 track updates.
trackPlotter(BEP, 'DisplayName','track', 'HistoryDepth',10);

axis(BEP.Parent, 'equal');
xlim(BEP.Parent, [-frontBackLim frontBackLim]);
ylim(BEP.Parent, [-40 40]);

% Add an outline plotter for ground truth
outlinePlotter(BEP, 'Tag', 'Ground truth');
end

%% Updates the plots in real-time

function updateBEP(BEP, egoCar, detections, confirmedTracks, psel, vsel)
% Update road boundaries and their display
rb = roadBoundaries(egoCar);
plotLaneBoundary(findPlotter(BEP,'DisplayName','road'),rb);

% update ground truth data
[position, yaw, length, width, originOffset, color] = targetOutlines(egoCar);
plotOutline(findPlotter(BEP,'Tag','Ground truth'), position, yaw, length, width, 'OriginOffset', originOffset, 'Color', color);

% Prepare and update detections display
N = numel(detections);
detPos = zeros(N,2);
isRadar = true(N,1);
for i = 1:N
    detPos(i,:) = detections{i}.Measurement(1:2)';
    if detections{i}.SensorIndex > 6 % Vision detections
        isRadar(i) = false;
    end
end
plotDetection(findPlotter(BEP,'DisplayName','vision'), detPos(~isRadar,:));
plotDetection(findPlotter(BEP,'DisplayName','radar'), detPos(isRadar,:));

% Prepare and update tracks display
trackIDs = {confirmedTracks.TrackID};
labels = cellfun(@num2str, trackIDs, 'UniformOutput', false);
[tracksPos, tracksCov] = getTrackPositions(confirmedTracks, psel);
tracksVel = getTrackVelocities(confirmedTracks, vsel);
plotTrack(findPlotter(BEP,'DisplayName','track'), tracksPos, tracksVel, tracksCov, labels);
end