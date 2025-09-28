%% Prediction NN
clc,clear all, close all
% Load Model
load Train1.mat

% Create Target Trajector
NumOfSample = 1e2;
TrajSelect = 'c';  % Square Traj -s  Half circle -c 
switch TrajSelect
    case 's'
        StartPoint = [100 160 100];
        P1 = [linspace(100,-100,NumOfSample)'  linspace(160,160,NumOfSample)' linspace(100,100,NumOfSample)'];
        P2 = [linspace(-100,-100,NumOfSample)' linspace(160,120,NumOfSample)' linspace(100,200,NumOfSample)'];
        P3 = [linspace(-100,100,NumOfSample)'  linspace(120,120,NumOfSample)' linspace(200,200,NumOfSample)'];
        P4 = [linspace(100,100,NumOfSample)'   linspace(120,160,NumOfSample)' linspace(200,100,NumOfSample)'];
        TestInput = [StartPoint; P1; P2; P3; P4]; % Target EEPose
    case 'c'
        cy = linspace(150,180,NumOfSample);
        cx = 50*sin(.5*cy);
        cz = 25*cos(.5*cy)+125;
        TestInput = [cx; cy; cz]'; % Target EEPose
    otherwise
        display("Choose eligible trajector!")
end

TrajPlot = TestInput;
TestInput = Normalization(TestInput,InputMax,InputMin);  

TestTheta = net(TestInput'); % NN IK Solution
TestTheta = DeNormalization(TestTheta',OutputMax,OutputMin); 

% Plot Robot NN Output
figure('units','normalized','outerposition',[0 0 1 1],'color','w')
% gif('NNTraj2.gif')
for k=1:size(TestTheta,1)

    clf
    trplot(eye(4),'frame','Base','thick',1,'rgb','length',50), hold on, grid on
    axis equal,axis([-350 350 -350 350 0 350])
    view(120,10)  % First arguman is azimuth angle, second is elevation angle

    theta = [TestTheta(k,:),0];
    Tee = eye(4,4);
    for i=1:size(theta,2)
        temp = Tee;
        T(:,:,i) = DHMatrixModify(alpha(i),a(i),d(i),theta(i));
        Tee = Tee * T(:,:,i);
        plot3([temp(1,4) Tee(1,4)],[temp(2,4) Tee(2,4)],[temp(3,4) Tee(3,4)],'k','LineWidth',1);
        xlabel('X-axis'),ylabel('Y-axis'),zlabel('Z-axis'),title('3DOF RRR Type Robot Inverse Kinematic Simulation via Neural Network')
        if i == 4
            trplot(Tee,'frame','EE',num2str(i),'thick',1,'rgb','length',50)
        else
            trplot(Tee,'frame',num2str(i),'thick',1,'rgb','length',50)
        end
    end
    NNEEPose(k,:) = Tee(1:3,4);
    NNPosePoint = Tee(1:3,4);
    Tee(find(abs(Tee)<1e-5)) = 0;
    strEE = num2str(Tee);
    text(-130,-300,300,strEE)
    plot3(TrajPlot(:,1),TrajPlot(:,2),TrajPlot(:,3),'k','Linewidth',2)
    plot3(NNEEPose(:,1),NNEEPose(:,2),NNEEPose(:,3),'r--','Linewidth',2)
    scatter3(NNPosePoint(1),NNPosePoint(2),NNPosePoint(3),'filled','o','MarkerEdgeColor','k','MarkerFaceColor',[0 .75 .75])
%     gif
    drawnow
end

