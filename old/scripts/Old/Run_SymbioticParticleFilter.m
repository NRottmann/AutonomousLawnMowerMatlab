% This a script in order to test the symbiotic particle filter
%
% Author: Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)
% Date: 05.11.2018

%% Clear everything
close all;
clear all;
clc;

%% Choose Parameters
pose = [-3; 0; -pi];                   % start pose
pose_store = pose;
numParticles = 500;
map_name = 'map_02.mat';         	% Choose the map which should be loaded
load(map_name); 
axVec = [polyMap.XWorldLimits polyMap.YWorldLimits];

%% Initialize classes
ctrl = WallFollower();
grassSensor = GrassSensor(polyMap);
odometryModel = OdometryModel();
wallFollower = WallFollower();
randomController = RandomController();
pf = SymbioticParticleFilter(numParticles,polyMap,[pose; 0],...
                            grassSensor,odometryModel,wallFollower,randomController);

%% Do the first plot
figure(1)
particles = getParticlePosition(pf);
h1 = scatter(particles(1,:),particles(2,:),5,'r', 'filled');
hold on
h2 = scatter(pose(1),pose(2),30,'g', 'filled');
hold on
h4 = plot(polyMap.x,polyMap.y,'k');     % polygon
h3 = plot(polyMap.x,polyMap.y,'k');
h5 = plot(polyMap.x,polyMap.y,'k');
h6 = plot(polyMap.x,polyMap.y,'k');
axis(axVec)

%% Get sensor information
out = get_config('Sensor');
posRight =  out.posRight;

%% Run the iteration
odometryData.deltaR1 = 0;
odometryData.deltaR2 = 0;
u = [0; 0];
tic

first = true;
second = true;
l_arrow = 0.2;

for i = 1:20000
    % Step 1: Get sensor measurements
    [sensorData] = measure(grassSensor,pose);
    
    % Step 2: Move Robot and store positions
    [pose, motionData] = kinModel(pose, u, true);
    
    % Step 3: Corrupt pose with noise
    [odometryModel,odometryData] = odometryModel.odometryData(pose, motionData);
    p_corrupted = odometryModel.odometryPose(pose,true,1);
    
    % Step 4: Particle Filter algorithm
    [pf,u] = update(pf,sensorData,odometryData,p_corrupted);
    
    % Step 5: Print actual outcome
    if pf.GlobalLocalization
%         delete(h3)
%         delete(h2)
%         delete(h1)
%         R = [cos(pose(3)) -sin(pose(3)); sin(pose(3)) cos(pose(3))];
%         sensorPosition = pose(1:2) + R*posRight;
%         h3 = scatter(sensorPosition(1),sensorPosition(2),30,'g', 'filled');
%         hold on
%         h2 = scatter(pose(1),pose(2),80,'g', 'filled');
%         hold on
%         h1 = scatter(pf.DP(1,:),pf.DP(2,:),20,'rx');
%         axis(axVec)
%         drawnow
    end
    if ~pf.GlobalLocalization && first
        pose_est = getMeanVariance(pf);
        delete(h6)
        delete(h5)
        delete(h3)
        delete(h2)
        delete(h1)
        particles = getParticlePosition(pf);
        h1 = scatter(particles(1,:),particles(2,:),5,'k', 'filled');
        hold on
        uu = l_arrow*cos(pose(3));
        vv = l_arrow*sin(pose(3));
        h5 = quiver(pose(1),pose(2),uu,vv,'m','LineWidth',5,'ShowArrowHead','on','MaxHeadSize',8.0);
        hold on
        h2 = scatter(pose(1),pose(2),100,'m', 'filled');
        hold on
        h3 = scatter(pose_est(1),pose_est(2),100,'c', 'filled');
        hold on
        uu = l_arrow*cos(pose_est(3));
        vv = l_arrow*sin(pose_est(3));
        h6 = quiver(pose_est(1),pose_est(2),uu,vv,'c','LineWidth',5,'ShowArrowHead','on','MaxHeadSize',8.0);
        axis(axVec)
        xlabel('$$x$$ in meter','Interpreter','latex')
        ylabel('$$y$$ in meter','Interpreter','latex')
        legend([h1 h2 h3],{'Particles','Real Pose','Estimated Pose'})
        drawnow
        pause
        first = false;
    end
    if ~pf.GlobalLocalization && pf.RandomControl && second
        pose_est = getMeanVariance(pf);
        delete(h6)
        delete(h5)
        delete(h3)
        delete(h2)
        delete(h1)
        particles = getParticlePosition(pf);
        h1 = scatter(particles(1,:),particles(2,:),5,'k', 'filled');
        hold on
        uu = l_arrow*cos(pose(3));
        vv = l_arrow*sin(pose(3));
        h5 = quiver(pose(1),pose(2),uu,vv,'m','LineWidth',5,'ShowArrowHead','on','MaxHeadSize',8.0);
        hold on
        h2 = scatter(pose(1),pose(2),100,'m', 'filled');
        hold on
        h3 = scatter(pose_est(1),pose_est(2),100,'c', 'filled');
        hold on
        uu = l_arrow*cos(pose_est(3));
        vv = l_arrow*sin(pose_est(3));
        h6 = quiver(pose_est(1),pose_est(2),uu,vv,'c','LineWidth',5,'ShowArrowHead','on','MaxHeadSize',8.0);
        axis(axVec)
        xlabel('$$x$$ in meter','Interpreter','latex')
        ylabel('$$y$$ in meter','Interpreter','latex')
%         legend([h1 h2 h3],{'Particles','Real Pose','Estimated Pose'})
        drawnow
        pause
        second = false;
    end
    if ~pf.GlobalLocalization && pf.RandomControl && ~second
        pose_est = getMeanVariance(pf);
        delete(h6)
        delete(h5)
        delete(h3)
        delete(h2)
        delete(h1)
        particles = getParticlePosition(pf);
        h1 = scatter(particles(1,:),particles(2,:),5,'k', 'filled');
        hold on
        uu = l_arrow*cos(pose(3));
        vv = l_arrow*sin(pose(3));
        h5 = quiver(pose(1),pose(2),uu,vv,'m','LineWidth',5,'ShowArrowHead','on','MaxHeadSize',8.0);
        hold on
        h2 = scatter(pose(1),pose(2),100,'m', 'filled');
        hold on
        h3 = scatter(pose_est(1),pose_est(2),100,'c', 'filled');
        hold on
        uu = l_arrow*cos(pose_est(3));
        vv = l_arrow*sin(pose_est(3));
        h6 = quiver(pose_est(1),pose_est(2),uu,vv,'c','LineWidth',5,'ShowArrowHead','on','MaxHeadSize',8.0);
        axis(axVec)
        xlabel('$$x$$ in meter','Interpreter','latex')
        ylabel('$$y$$ in meter','Interpreter','latex')
%         legend([h1 h2 h3],{'Particles','Real Pose','Estimated Pose'})
        drawnow
        second = false;
    end
    
    % Store Pose
    pose_store = pose;
end

%% Plot correlation
figure(2)
plot(pf.Corr,'k*')
xlabel('Vertex index','Interpreter','latex')
ylabel('$$c$$','Interpreter','latex')
axis([0 17 0 16])
box off
