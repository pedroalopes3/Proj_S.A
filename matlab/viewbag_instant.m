function plot_trajectory_with_sonar()
    % Define the data directory and bag name
    directory = '/Users/pedro/Documents/IST/3_Ano/SAut/Proj_S.A/bags';
    bagname = '00bag-29-4.24.bag';
    path = fullfile(directory, bagname);

    % Load the bag file
    bag = rosbag(path);

    % Select topics from the bag
    bSel_pose = select(bag,'Topic','/pose');
    bSel_sonar = select(bag,'Topic','/sonar');

    % Read messages from selected topics
    msgStructs_pose = readMessages(bSel_pose,'DataFormat','struct');
    msgStructs_sonar = readMessages(bSel_sonar,'DataFormat','struct');

    % Plot the trajectory
    xPoints = cellfun(@(m) double(m.Pose.Pose.Position.X), msgStructs_pose);
    yPoints = cellfun(@(m) double(m.Pose.Pose.Position.Y), msgStructs_pose);
    figure;
    plot(xPoints,yPoints,'b','LineWidth', 2);
    hold on;

    % Plot the sonar point cloud
    sonarX = zeros(length(msgStructs_sonar), 8);
    sonarY = zeros(length(msgStructs_sonar), 8);
    rangePointsCell = cellfun(@(m) double(m.Ranges), msgStructs_sonar, 'UniformOutput', false);
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

    xlabel('X');
    ylabel('Y');
    legend('Robot Trajectory', 'Sonar Point Cloud');

    % Add slider for selecting position on trajectory
    uicontrol('Style', 'slider', 'Min', 1, 'Max', length(xPoints), ...
              'Value', 1, 'Position', [400, 50, 120, 20], ...
              'Callback', @updatePlot);

    % Callback function for slider
    function updatePlot(src, ~)
        % Get the selected position on the trajectory
        position = round(src.Value);

        % Update the trajectory plot
        plot(xPoints(position), yPoints(position), 'ro', 'MarkerSize', 8);

        % Update the sonar point cloud plot
        rangePoints = rangePointsCell{position};
        sonarX = xPoints(position) + rangePoints .* cosd(sensorAngles);
        sonarY = yPoints(position) + rangePoints .* sind(sensorAngles);
        scatter(sonarX, sonarY, 'g', 'filled');
    end

    % Enable clicking on the plot to display all data points
    set(gcf, 'ButtonDownFcn', @plotAllData);

    % Callback function for clicking on the plot
    function plotAllData(~, ~)
        % Plot the full trajectory
        plot(xPoints, yPoints, 'b');

        % Plot the full sonar point cloud
        for i = 1:length(xPoints)
            rangePoints = rangePointsCell{i};
            sonarX = xPoints(i) + rangePoints .* cosd(sensorAngles);
            sonarY = yPoints(i) + rangePoints .* sind(sensorAngles);
            scatter(sonarX, sonarY, 'r', 'filled');
        end
    end
end
