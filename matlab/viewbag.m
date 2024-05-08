directory = '/Users/pedro/Documents/IST/3_Ano/SAut/Proj_S.A/bags';
bagname = '00bag-07-5.24.bag';
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
                laserX(i,j) = xPoints(i) + rangePoints_laser(j) * cosd(laserAngles(j)-180);
                laserY(i,j) = yPoints(i) + rangePoints_laser(j) * sind(laserAngles(j)-180);
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
    
    % sonarPlot = scatter(sonarX(:), sonarY(:), 'r', 'filled');
    % sonarPlot.SizeData = 5; % to adjust the points thickness
    
    % set plot labels
    xlabel('X');
    ylabel('Y');
    legend('Robot Trajectory', 'Laser Point Cloud','Sonar Point Cloud');

elseif Mode == 1
    
    exit = 0;
    while exit == 0

        Num_instants = length(msgStructs_pose);
        instant = 0;
        disp(['Choose an instant to plot the sensors data, from 0 to ', num2str(Num_instants)]);
        disp(['(If you want to exit the program, enter ', num2str(Num_instants+1), ')']);
        instant = input(': ') + 1;
        disp(instant);
        
        if instant == Num_instants+1
            exit = 1;
            continue
        end

        % plot the trajectory
        xPoints = cellfun(@(m) double(m.Pose.Pose.Position.X),msgStructs_pose);
        yPoints = cellfun(@(m) double(m.Pose.Pose.Position.Y),msgStructs_pose);
        xInstant = xPoints(instant);
        yInstant = yPoints(instant);
        figure;
        posPlot = scatter(xInstant, yInstant, 'b', 'filled');
        posPlot.SizeData = 200;
        hold on;
        
        % plot the laser point cloud
        laserX = xInstant;
        laserY = yInstant;
        rangePointsCell_laser = cellfun(@(m) double(m.Ranges),msgStructs_scan,'UniformOutput',false);
        start_angle = -30;
        increment_angle = 0.35156;
        num_samples = 726;
        laserAngles = linspace(start_angle, start_angle + increment_angle * (num_samples - 1), num_samples);
        rangePoints_laser = rangePointsCell_laser{instant};
        for j = 1:length(laserAngles)
                laserX(j) = xInstant + rangePoints_laser(j) * cosd(laserAngles(j));
                laserY(j) = yInstant + rangePoints_laser(j) * sind(laserAngles(j));
        end
        
        laserPlot = scatter(laserX, laserY, 'y', 'filled');
        laserPlot.SizeData = 10; % to adjust the points thickness
        hold on;
    
        
        % plot the sonar point cloud
        sonarX = xInstant;
        sonarY = yInstant;
        rangePointsCell = cellfun(@(m) double(m.Ranges),msgStructs_sonar,'UniformOutput',false);
        sensorAngles = [0, 40, 60, 80, 100, 120, 140, 180];
        rangePoints = rangePointsCell{instant};
        for j = 1:length(sensorAngles)
                sonarX(j) = xInstant + rangePoints(j) * cosd(180-sensorAngles(j));
                sonarY(j) = yInstant + rangePoints(j) * sind(sensorAngles(j));
        end
        
        sonarPlot = scatter(sonarX, sonarY, 'r', 'filled');
        sonarPlot.SizeData = 10; % to adjust the points thickness
        
        % set plot labels
        xlabel('X');
        ylabel('Y');
        legend('Robot`s Location', 'Laser Point Cloud','Sonar Point Cloud');
    
    end

end