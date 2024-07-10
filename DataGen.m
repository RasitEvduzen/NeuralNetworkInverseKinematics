clc,clear all,close all,warning off;
% 3DOF Robotic Arm Inverse Kinematics
% 29-Feb-2024
% Written By Rasit.

%% Create Robot Forward Kinematic Simulation
%---------------------- Robot DH Parameters ----------------------%
a2 = 100; a3 = 100; d1 = 100;   % Robot Link Lenght
a     = [0,0,a2,a3];
alpha = [0,90,0,90];
d     = [d1,0,0,0];

NofData = 1e5;
qmax = [180 90 -90];    % Good Result-1 [180 90 -90];
qmin = [0 0 0];         % Good Result-1 [0 0 0];
ThetaRand = [qmax(1)*rand(NofData,1)-qmin(1) qmax(2)*rand(NofData,1)-qmin(2) qmax(3)*rand(NofData,1)-qmin(3)];


EEPose = zeros(NofData,3);
SimPlot = 0;
if  SimPlot == 1
    figure('units','normalized','outerposition',[0 0 1 1],'color','w')
%     gif('NNTraj1DataGen.gif')
end

for j=1:NofData

    if SimPlot == 1
        clf
        trplot(eye(4),'frame','Base','thick',1,'rgb','length',50), hold on, grid on
        axis equal,axis([-350 350 -350 350 0 350])
        view(145,20)  % First arguman is azimuth angle, second is elevation angle
    end

    theta = [ThetaRand(j,1),ThetaRand(j,2),ThetaRand(j,3),0];
    Tee = eye(4,4);
    for i=1:size(theta,2)

        temp = Tee;
        T(:,:,i) = DHMatrixModify(alpha(i),a(i),d(i),theta(i));
        Tee = Tee * T(:,:,i);

        if SimPlot == 1
            plot3([temp(1,4) Tee(1,4)],[temp(2,4) Tee(2,4)],[temp(3,4) Tee(3,4)],'k','LineWidth',1);
            xlabel('X-axis'),ylabel('Y-axis'),zlabel('Z-axis'),title('3DOF RRR Type Robot Simulation')
            if i == 4
                trplot(Tee,'frame','EE',num2str(i),'thick',1,'rgb','length',50)
            else
                trplot(Tee,'frame',num2str(i),'thick',1,'rgb','length',50)
            end
        end
    end
    EEPose(j,:) = Tee(1:3,4);
    if SimPlot == 1
        Tee(find(abs(Tee)<1e-5)) = 0;
        strEE = num2str(Tee);
        text(-130,-300,300,strEE)
%         gif
        drawnow
    end
    if j == NofData
     scatter3(EEPose(:,1),EEPose(:,2),EEPose(:,3),'filled','o','MarkerEdgeColor','k','MarkerFaceColor',[1 0 0])
     hold on, xlabel('X-axis'),ylabel('Y-axis'),zlabel('Z-axis'),axis equal
    end
end

%% Create Data for Training & Testing
Normalization   = @(data,max,min) ((data - min)./(max-min)); % Max-Min Normalization
DeNormalization = @(data,max,min) ((data.*(max-min))+min);

Input = EEPose;
InputMax = max(Input);
InputMin = min(Input);

Output = ThetaRand;
OutputMax = max(Output);
OutputMin = min(Output);

% Normalization
Input = Normalization(Input,InputMax,InputMin);
Output = Normalization(Output,OutputMax,OutputMin);

%% Train NN
return
hiddenLayerSize = 20;
TrainNN
% Save Model 
% save Train1 net InputMax InputMin OutputMax OutputMin a alpha d Normalization DeNormalization

