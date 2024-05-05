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


Mode = 1;

if Mode == 0

    % plot the trajectory
    xPoints = cellfun(@(m) double(m.Pose.Pose.Position.X),msgStructs_pose);
    yPoints = cellfun(@(m) double(m.Pose.Pose.Position.Y),msgStructs_pose);
    plot(xPoints,yPoints,'b','LineWidth', 2);
    hold on;
    
    % plot the laser point cloud
    laserX = xPoints;
    laserY = yPoints;
    rangePointsCell_laser = cellfun(@(m) double(m.Ranges),msgStructs_scan,'UniformOutput',false);
    start_angle = -30;
    increment_angle = 0.35156;
    num_samples = 726;
    laserAngles = linspace(start_angle, start_angle + increment_angle * (num_samples - 1), num_samples);
    for i = 1:length(xPoints)
        rangePoints_laser = rangePointsCell_laser{i};
        for j = 1:length(laserAngles)
            if rangePoints_laser(j) < 4
                laserX(i,j) = xPoints(i) + rangePoints_laser(j) * cosd(laserAngles(j));
                laserY(i,j) = yPoints(i) + rangePoints_laser(j) * sind(laserAngles(j));
            else
                laserX(i,j) = NaN;  
                laserY(i,j) = NaN;
            end
        end    
    end
    
    laserPlot = scatter(laserX(:), laserY(:), 'y', 'filled');
    laserPlot.SizeData = 1; % to adjust the points thickness
    hold on;
    
    
    % plot the sonar point cloud
    sonarX = xPoints;
    sonarY = yPoints;
    rangePointsCell = cellfun(@(m) double(m.Ranges),msgStructs_sonar,'UniformOutput',false);
    sensorAngles = [0, 40, 60, 80, 100, 120, 140, 180];
    for i = 1:length(xPoints)
        rangePoints = rangePointsCell{i};
        for j = 1:length(sensorAngles)
            if rangePoints(j) < 5
                sonarX(i,j) = xPoints(i) + rangePoints(j) * cosd(180-sensorAngles(j));
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
    legend('Robot Trajectory', 'Laser Point Cloud','Sonar Point Cloud');

elseif Mode == 1

    Num_instants = length(msgStructs_pose);
    instant = 0;
    disp(['Choose an instant to plot the sensors data, from 0 to', num2str(Num_instants)]);
    instant = input(': ');
    disp(instant);
    
    % plot the trajectory
    xPoints = cellfun(@(m) double(m.Pose.Pose.Position.X),msgStructs_pose);
    yPoints = cellfun(@(m) double(m.Pose.Pose.Position.Y),msgStructs_pose);
    plot(xPoints(instant),yPoints(instant),'b','LineWidth', 10);
    hold on;
    
    % plot the laser point cloud
    laserX = xPoints;
    laserY = yPoints;
    rangePointsCell_laser = cellfun(@(m) double(m.Ranges),msgStructs_scan,'UniformOutput',false);
    start_angle = -30;
    increment_angle = 0.35156;
    num_samples = 726;
    laserAngles = linspace(start_angle, start_angle + increment_angle * (num_samples - 1), num_samples);
    for i = 1:length(xPoints)
        rangePoints_laser = rangePointsCell_laser{i};
        for j = 1:length(laserAngles)
            if rangePoints_laser(j) < 4
                laserX(i,j) = xPoints(i) + rangePoints_laser(j) * cosd(laserAngles(j));
                laserY(i,j) = yPoints(i) + rangePoints_laser(j) * sind(laserAngles(j));
            else
                laserX(i,j) = NaN;  
                laserY(i,j) = NaN;
            end
        end    
    end
    
    laserPlot = scatter(laserX(instant), laserY(instant), 'y', 'filled');
    laserPlot.SizeData = 100; % to adjust the points thickness
    hold on;
    
    
    % plot the sonar point cloud
    sonarX = xPoints;
    sonarY = yPoints;
    rangePointsCell = cellfun(@(m) double(m.Ranges),msgStructs_sonar,'UniformOutput',false);
    sensorAngles = [0, 40, 60, 80, 100, 120, 140, 180];
    for i = 1:length(xPoints)
        rangePoints = rangePointsCell{i};
        for j = 1:length(sensorAngles)
            if rangePoints(j) < 5
                sonarX(i,j) = xPoints(i) + rangePoints(j) * cosd(180-sensorAngles(j));
                sonarY(i,j) = yPoints(i) + rangePoints(j) * sind(sensorAngles(j));
            else
                sonarX(i,j) = NaN;  
                sonarY(i,j) = NaN;
            end
        end    
    end
    
    sonarPlot = scatter(sonarX(instant), sonarY(instant), 'r', 'filled');
    sonarPlot.SizeData = 100; % to adjust the points thickness
    
    % set plot labels
    xlabel('X');
    ylabel('Y');
    legend('Robot Trajectory', 'Laser Point Cloud','Sonar Point Cloud');    
end