clc;
clearvars;
close all;
format long;

% Add paths
path = genpath('vrep_utils');
addpath(path);

%% For RCM & joint limits study
% The green ball is located at (+6.4945e-01, -1.7500e-01, +1.5000e-1) in VREP

%% Lengths of the shaft, the flexible segment and the camera
shaftLen = 0.377;
flexLen = 0.049;
cameraLen = 0.016;

%% Lengths of the disks
Ldisk2 = 0.004;
Ldisk3 = 0.009;
Ldisk4 = 0.014;
Ldisk5 = 0.019;
Ldisk6 = 0.024;
Ldisk7 = 0.029;
Ldisk8 = 0.034;
Ldisk9 = 0.039;
Ldisk10 = 0.044;

% Robot parameters
numJoints = 7; % Number of robot joints
% q = [0.0; pi/12.0; 0.0; -2.0*pi/3.0; 0.0; 5.0*pi/6.0; 0.0]; % Joint states in V-REP
q = [0; -pi/4; 0; -3 * pi/4; 0; pi/2; 0];
jointOffset = [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];

qDH = q + jointOffset; % Joint states in DH sense
phi = 0.0;
theta = 0.0;

m = 5; % Pixel coordinates + RCM postions
n = numJoints + 3; % Plus two DoFs of the flexible joint and one virtual insertion DoF
dq = zeros(n, 1);

% Joint angle limits (rad) in V-REP
qMax = [166; 101; 166; -4; 166; 215; 166]*pi/180;
qMin = [-166; -101; -166; -176; -166; -1; -166]*pi/180;

% Joint angle limits (rad) in DH sense
qMax = qMax + jointOffset;
qMin = qMin + jointOffset; 

% Joint velocity limits (rad/s) in V-REP
dqMax =  [150; 150; 150; 150; 180; 180; 180]*pi/180;
dqMin = -dqMax;

% Limits of the flexible joint and the virtual insertion DoF
qMin = [qMin;  -2*pi; -pi/3; 0.3]; % 1e6
qMax = [qMax; 2*pi; pi/3; 0.7];

dqMin = [dqMin; -pi; -pi; -1.0];
dqMax = [dqMax; pi; pi; 1.0];

% VREP simulation
totalTime = 10.0;
timeStep = 0.01;
simuTime = 0.0;

% Neural algoritm
tau = 0.01;
iterNum = 5000;
alpha = 1.0; % Joint limit parameter in DNN
gamma = 0.3; % DNN parameter
r = 0.5; % Activation parameter
kxi = 300; % Joint limit handling parameter
rcmBool = 1; % With or without RCM constraints
caseFun = 4; % Activation function selection

% Neural state
mu = zeros(m + n, iterNum);
mu0 = mu(:, 1);

% Gains
gain = 100.0;
K = diag([3.0; 3.0; 5.0; 5.0; 5.0]); % 3.0 for linear case 
M = diag([1.0/gain; 1.0/gain; 1.0/gain; 1.0/gain; 1.0/gain; 1.0/gain; 1.0/gain; 1.0; 1.0; 1.0/gain]);

% Monitoring error
nerr = zeros(iterNum, 1);

% Camera parameters
f = 800; % Focal length
rho = 1.0; % Pixel width
Z = 0.3; % Constant depth
resolutionX = 640;
resolutionY = 480; 
u0 = resolutionX/2;
v0 = resolutionY/2;
utarget = u0;
vtarget = v0;

%% Pose of the trocar: to be estimated in practice
beta = 0.5;
d8 = beta*shaftLen;
Htrocar = robotKinM(qDH, d8);
ptrocar = Htrocar(1:3, 4);
z = [qDH; phi; theta; beta];

disp('Program started');
vrep = remApi('remoteApi');
vrep.simxFinish(-1);
clientID = vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);

if (clientID > -1)
    disp('Connected to remote API server');
    
    % Synchronous mode
    vrep.simxSynchronous(clientID, true);
    
    % Get robot handles
    for i = 1:numJoints
        motorName = ['Panda_joint' num2str(i)];
        [returnCode, handle] = vrep.simxGetObjectHandle(clientID, motorName, vrep.simx_opmode_blocking);
        if returnCode ~= vrep.simx_return_ok
            disp(['Handle of motor ' motorName ' is not found!']);
        else
            disp(['Find motor ' motorName]);
        end
    end
    
    % Get target handle
    objectName = 'Target';
    [returnCode, targetHandle] = vrep.simxGetObjectHandle(clientID, objectName, vrep.simx_opmode_blocking);
    if returnCode ~= vrep.simx_return_ok
        disp(['Handle of ' objectName ' is not found!']);
    else
        disp(['Find object ' objectName]);
    end
    
    % Get camera handles
    objectName = 'blobDetectionCamera_camera';
    [returnCode, cameraHandle] = vrep.simxGetObjectHandle(clientID, objectName, vrep.simx_opmode_blocking);
    if returnCode ~= vrep.simx_return_ok
        disp(['Handle of ' objectName ' is not found!']);
    else
        disp(['Find object ' objectName]);
    end
    
    [returnCode, detectionState, auxData, auxPacketInfo] = vrep.simxReadVisionSensor(clientID, cameraHandle, vrep.simx_opmode_streaming);
    
    % Start simulation
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);
    pause(3);
    
    i = 1;
    while (vrep.simxGetConnectionId(clientID) ~= -1 && simuTime <= totalTime)
        
        % Pose of the shaft's end point (end tip of disk 1) with respect to the world
        d8 = shaftLen + 0.002;
        H08 = robotKinM(qDH, d8);
        R08 = H08(1:3, 1:3);
        P08 = H08(1:3, 4);
        
        % Pose of the shaft's start point (robot flange) with respect to the world
        Htemp = [1 0 0 0;...
            0 1 0 0;...
            0 0 1 -d8;...
            0 0 0 1];
        H07 = H08*Htemp;
        P07 = H07(1:3, 4);

        % Pose of the camera tip with respect to the world
        toolAng = z(8:9);
        Htemp = softH(toolAng, flexLen);
        H09 = H08*Htemp;
        Htemp = [1 0 0 0;...
            0 1 0 0;...
            0 0 1 cameraLen;...
            0 0 0 1];
        H09 = H09*Htemp;
        R09 = H09(1:3, 1:3);
        P09 = H09(1:3, 4);

        % Jacobian matrix of the camera tip with respect to the shaft's end point (end tip of disk 1)
        Jinst = Jtool(toolAng, flexLen, cameraLen);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
        %% Only for simulation  
        % Pose of the camera with respect to the world
        Htemp = [1 0 0 0;...
            0 1 0 0;...
            0 0 1 -cameraLen/2;...
            0 0 0 1];
        Hcamera = H09*Htemp;
        cameraData = reshape(Hcamera(1:3, :), 12, 1);

        % Pose of disk 2 with respect to the world 
        ratio = Ldisk2 / flexLen;
        toolAng(2) = ratio*theta;
        Htemp = softH(toolAng, ratio*flexLen);
        Hdisk = H08*Htemp;
        disk2Data = reshape(Hdisk(1:3, :), 12, 1);
        
        ratio = Ldisk3 / flexLen;
        toolAng(2) = ratio*theta;
        Htemp = softH(toolAng, ratio*flexLen);
        Hdisk = H08*Htemp;
        disk3Data = reshape(Hdisk(1:3, :), 12, 1);
        
        ratio = Ldisk4 / flexLen;
        toolAng(2) = ratio*theta;
        Htemp = softH(toolAng, ratio*flexLen);
        Hdisk = H08*Htemp;
        disk4Data = reshape(Hdisk(1:3, :), 12, 1);
        
        ratio = Ldisk5 / flexLen;
        toolAng(2) = ratio*theta;
        Htemp = softH(toolAng, ratio*flexLen);
        Hdisk = H08*Htemp;
        disk5Data = reshape(Hdisk(1:3, :), 12, 1);
        
        ratio = Ldisk6 / flexLen;
        toolAng(2) = ratio*theta;
        Htemp = softH(toolAng, ratio*flexLen);
        Hdisk = H08*Htemp;
        disk6Data = reshape(Hdisk(1:3, :), 12, 1);
        
        ratio = Ldisk7 / flexLen;
        toolAng(2) = ratio*theta;
        Htemp = softH(toolAng, ratio*flexLen);
        Hdisk = H08*Htemp;
        disk7Data = reshape(Hdisk(1:3, :), 12, 1);
        
        ratio = Ldisk8 / flexLen;
        toolAng(2) = ratio*theta;
        Htemp = softH(toolAng, ratio*flexLen);
        Hdisk = H08*Htemp;
        disk8Data = reshape(Hdisk(1:3, :), 12, 1);
        
        ratio = Ldisk9 / flexLen;
        toolAng(2) = ratio*theta;
        Htemp = softH(toolAng, ratio*flexLen);
        Hdisk = H08*Htemp;
        disk9Data = reshape(Hdisk(1:3, :), 12, 1);
        
        ratio = Ldisk10 / flexLen;
        toolAng(2) = ratio*theta;
        Htemp = softH(toolAng, ratio*flexLen);
        Hdisk = H08*Htemp;
        disk10Data = reshape(Hdisk(1:3, :), 12, 1);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
        %% Update simulated scene
        stateData = [qDH - jointOffset; disk2Data; disk3Data; disk4Data;...
            disk5Data; disk6Data; disk7Data; disk8Data; disk9Data; disk10Data; cameraData];

        [returnCode, ~, ~, ~, ~] = vrep.simxCallScriptFunction(clientID, 'Panda', vrep.sim_scripttype_childscript, 'updateState_function', [], stateData, [], [], vrep.simx_opmode_blocking);
    
        if (returnCode == vrep.simx_return_ok)
            disp(['Call remote function successfully and simuTime is ' num2str(simuTime)]);
        else
            disp('Failed to call remote function!');
        end
        
        if simuTime == 0
            % Set initial states of the simulated robot in V-REP
            disp('Set initial states of the simulated robot in V-REP');
            pause(0.001);
        else
            pause(0.001);
        end
        
        [returnCode, detectionState, auxData, auxPacketInfo] = vrep.simxReadVisionSensor(clientID, cameraHandle, vrep.simx_opmode_buffer);
        blobData = auxData;
        if (size(blobData) < 8)
            continue;
        end
        u = resolutionX - blobData(5)*resolutionX;
        v = blobData(6)*resolutionY;
        % disp([u, v]);
        
        % Image Jacobian matrix
        Jimg = Jimage(u, v, u0, v0, f, rho, Z);
        
        % Jacobian matrix of the shaft's start and end points with respect to the world
        [J7, Jr] = JrobotM(qDH, d8);
        J8 = Jr(1:3, :);

        % Jacobian matrix of the camera tip with respect to the world
        Sp = [hat(P08 - P09); zeros(3, 3)];
        Rp = [R08, zeros(3, 3); zeros(3, 3), R08];
        Jsystem = [Jr + Sp*Jr(4:6, :), Rp*Jinst];
        
        % Jacobian matrix for visual servoing
        Jtotal = Jimg*[transpose(R09), zeros(3,3); zeros(3,3), transpose(R09)]*Jsystem;
                     
        Jaug = [J7 + beta*(J8 - J7), zeros(3, 2), P08 - P07];
        Jfinal = [Jtotal*1e-3, zeros(size(Jtotal, 1), 1); Jaug];
        prcm = P07 + beta*(P08 - P07);
        
        %% Neural algorithm
        k = 1;
        errTol = 10;
        tic
        while (k <= iterNum) && (errTol >= 1e-3)
            if (k == 1)
                tempJ = [Jfinal; eye(n, n)];
                
                mu(:, k) = mu0;
                
                b = K*[(utarget - u)*1e-3; (vtarget - v)*1e-3; rcmBool*(ptrocar - prcm)];          
                d = [max(dqMin, kxi*(qMin - z)); min(dqMax, kxi*(qMax - z))];
                
                zL = [b; d(1 : n)];
                zU = [b; d(n + 1 : end)];
                dq = -M*(transpose(tempJ)*mu(:, k));
                
                nerr(k) = 10;
            else
                x = -alpha*tempJ*dq;              
                y = gFun(alpha, mu(:, k - 1) - x, zL, zU);
                y = y + x;
                
                mu(:, k) = mu(:, k - 1) - tau*gamma*actFun(y, r, caseFun);
                
                dq = -M*(transpose(tempJ)*mu(:, k));
                nerr(k) = norm(y);
            end          
            errTol = nerr(k);
            k = k + 1;
        end
        toc
        mu0 = mu(:, k - 1);
        disp(k - 1);
        disp(errTol);
        
        imagData(i, 1) = u;
        imagData(i, 2) = v;
        jointAngle(i, :) = [qDH - jointOffset; phi; theta; beta]; % VREP joint angles!
        jointVelocity(i, :) = dq; 
        t(i) = simuTime;
        rcmData(i, :) = transpose(prcm);
        solData(i, :) = errTol;
        cmpTime(i, :) = toc;
        numIteration(i, :) = k - 1;
        
        simuTime = simuTime + timeStep;
        
        qDH = qDH + dq(1:7)*timeStep;
        phi = phi + dq(8)*timeStep;
        theta = theta + dq(9)*timeStep;
        beta = beta + dq(10)*timeStep;
        
        z = [qDH; phi; theta; beta];
        
        % Trigger the next simulation step
        vrep.simxSynchronousTrigger(clientID);
        
        i = i + 1;
    end
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot);
    [returnCode, pingTime] = vrep.simxGetPingTime(clientID);
    vrep.simxFinish(clientID);
    disp('Connection is closed!')
else
    disp('Failed connecting to remote API server');
end
vrep.delete();
disp('Task is finished and program ended');

save data.mat imagData t jointAngle jointVelocity ptrocar rcmData solData cmpTime numIteration

load('data.mat')
axe = axes('XAxisLocation', 'bottom', 'YAxisLocation','right', 'Color','none', 'XColor', 'k','YColor', 'k');
line(imagData(:, 1), imagData(:, 2), 'Color', 'k', 'Parent', axe);
box on
grid on
xlim([0, 640])
ylim([0, 480])
hold on
plot(imagData(1, 1), imagData(1, 2), '*')
hold on
plot(imagData(end, 1), imagData(end, 2), '*')
set (gca, 'xdir', 'reverse' )

figure;
box on
hold on
plot(t, imagData(:, 1) - 320);
plot(t, imagData(:, 2) - 240);

figure;
plot(t, jointAngle);

figure;
plot(t, jointVelocity);

figure;
plot(t, rcmData' - ptrocar*ones(size(t)));

figure;
stem(t, solData, '.')
ylim([0, 1.2e-3])

figure;
stem(t, cmpTime, '.')

figure;
stem(t, numIteration, '.')
