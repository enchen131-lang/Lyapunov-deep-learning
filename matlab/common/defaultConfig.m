function cfg = defaultConfig()
%DEFAULTCONFIG Default settings for Visual_IBVS_Baseline and its helpers.
%
%   cfg = defaultConfig() returns the configuration consumed by the
%   baseline experiment and the shared kinematics/solver functions.

cfg.sim.totalTime = 10.0;
cfg.sim.timeStep = 0.05;
cfg.sim.initialTime = 0.0;

cfg.model.flexLen = 0.049;

cfg.geometry.shaftLen = 0.377;
cfg.geometry.flexLen = 0.049;
cfg.geometry.cameraLen = 0.016;
cfg.geometry.diskLengths = [0.004; 0.009; 0.014; 0.019; 0.024; ...
    0.029; 0.034; 0.039; 0.044];

cfg.robot.numJoints = 7;
cfg.robot.q0 = [0; -pi/4; 0; -3*pi/4; 0; pi/2; 0];
cfg.robot.jointOffset = zeros(7, 1);
cfg.robot.phi0 = 0.0;
cfg.robot.theta0 = 0.0;

qMax = [166; 101; 166; -4; 166; 215; 166]*pi/180 + cfg.robot.jointOffset;
qMin = [-166; -101; -166; -176; -166; -1; -166]*pi/180 + cfg.robot.jointOffset;
dqMax = [150; 150; 150; 150; 180; 180; 180]*pi/180;
dqMin = -dqMax;

cfg.limits.qMin = [qMin; -pi/3; -pi/3; 0.3];
cfg.limits.qMax = [qMax; pi/3; pi/3; 0.7];
cfg.limits.dqMin = [dqMin; -pi; -pi; -1.0];
cfg.limits.dqMax = [dqMax; pi; pi; 1.0];

cfg.camera.f = 800;
cfg.camera.rho = 1.0;
cfg.camera.Z = 0.3;
cfg.camera.resolutionX = 640;
cfg.camera.resolutionY = 480;
cfg.camera.u0 = cfg.camera.resolutionX/2;
cfg.camera.v0 = cfg.camera.resolutionY/2;

% Pixel path from Reference/trajectory.m (executable formula).
cfg.trajectory.amplitude = 200; % pixels
cfg.trajectory.period = 20.0; % s; default tracking duration
cfg.trajectory.center = [320; 240]; % [u; v], pixels (not camera principal point)

cfg.rcm.enabled = true;
cfg.rcm.beta0 = 0.5;
cfg.rcm.gain = diag([5.0; 5.0; 5.0]);

cfg.control.visualScale = 1e-3;
% cfg.control.Kimg = diag([22.0; 30.0]);
cfg.control.Kimg = diag([8;35]);
% U:22-11
%% V:30-15-8-6
%pid = 6,3
cfg.control.phiReference = 0.0; % rad
cfg.control.phiCenterGain = 0.3; % 1/s
gain = 100.0;
cfg.neural.tau = 0.01;
cfg.neural.iterNum = 50000;
cfg.neural.errTol = 1e-5;
cfg.neural.alpha = 1.0;
cfg.neural.gamma = 0.3;
cfg.neural.r = 0.5;
cfg.neural.caseFun = 4;
cfg.neural.kxi = 300;
cfg.neural.limitStepFraction = 0.5; % Maximum fraction of limit distance per step
cfg.neural.M = diag([1/gain; 1/gain; 1/gain; 1/gain; 1/gain; ...
    1/gain; 1/gain; 0.1; 1.0; 1/gain]);

cfg.vrep.address = '127.0.0.1';
cfg.vrep.port = 19999;
cfg.vrep.waitUntilConnected = true;
cfg.vrep.doNotReconnectOnceDisconnected = true;
cfg.vrep.timeOutInMs = 5000;
cfg.vrep.commThreadCycleInMs = 5;
cfg.vrep.startPause = 3.0;
cfg.vrep.stepPause = 0.001;
cfg.vrep.noBlobPause = 0.001;
cfg.vrep.maxNoBlobSteps = 200;

cfg.output.saveFile = 'data_clibvs.mat';
cfg.output.plot = true;
cfg.output.verbose = true;

end
