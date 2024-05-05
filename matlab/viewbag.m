directory = '/Users/pedro/Documents/IST/3_Ano/SAut/Proj_S.A/bags';
bagname = '00bag-29-4.24.bag';
path = fullfile(directory, bagname);

bag = rosbag(path);

% select topics from bag
bSel_pose = select(bag,'Topic','/pose');
bSel_scan = select(bag,'Topic','/scan');
bSel_sonar = select(bag,'Topic','/sonar');
bSel_tf = select(bag,'Topic','/tf');

% read messages from selected topics
msgStructs_pose = readMessages(bSel_pose,'DataFormat','struct');
msgStructs_scan = readMessages(bSel_scan,'DataFormat','struct');
msgStructs_sonar = readMessages(bSel_sonar,'DataFormat','struct');
msgStructs_tf = readMessages(bSel_tf,'DataFormat','struct');
msgStructs_sonar{1}

% plot the trajectory
xPoints = cellfun(@(m) double(m.Pose.Pose.Position.X),msgStructs_pose);
yPoints = cellfun(@(m) double(m.Pose.Pose.Position.Y),msgStructs_pose);
plot(xPoints,yPoints,'b','LineWidth', 2);
hold on;

% plot the laser point cloud
laserX = xPoints;
laserY = yPoints;
rangePointsCell = cellfun(@(m) double(m.Ranges),msgStructs_scan,'UniformOutput',false);
laserAngles = [0, 40, 60, 80, 100, 120, 140, 180];
for i = 1:length(xPoints)
    rangePoints = rangePointsCell{i};
    for j = 1:length(sensorAngles)
        if rangePoints(j) < 5
            sonarX(i,j) = xPoints(i) + rangePoints(j) * cosd(sensorAngles(j));
            sonarY(i,j) = yPoints(i) + rangePoints(j) * sind(sensorAngles(j));
        else
            sonarX(i,j) = NaN;  
            sonarY(i,j) = NaN;
        end
    end    
end


% plot the sonar point cloud
sonarX = xPoints;
sonarY = yPoints;
rangePointsCell = cellfun(@(m) double(m.Ranges),msgStructs_sonar,'UniformOutput',false);
sensorAngles = [0, 40, 60, 80, 100, 120, 140, 180];
for i = 1:length(xPoints)
    rangePoints = rangePointsCell{i};
    for j = 1:length(sensorAngles)
        if rangePoints(j) < 5
            sonarX(i,j) = xPoints(i) + rangePoints(j) * cosd(sensorAngles(j));
            sonarY(i,j) = yPoints(i) + rangePoints(j) * sind(sensorAngles(j));
        else
            sonarX(i,j) = NaN;  
            sonarY(i,j) = NaN;
        end
    end    
end

sonarPlot = scatter(sonarX(:), sonarY(:), 'r', 'filled');
sonarPlot.SizeData = 5; % to adjust the points thickness

% set plot labels
xlabel('X');
ylabel('Y');
legend('Robot Trajectory', 'Sonar Point Cloud');

