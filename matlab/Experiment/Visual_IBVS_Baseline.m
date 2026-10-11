function results = Visual_IBVS_Baseline(config)
%FIXEDTRAJECTORYWITHOUTCL Nominal-model pixel tracking with RCM, without learning.
% From matlab: addpath('Experiment'); results = Visual_IBVS_Baseline();
% Based on Panda_RCM_RNN3_insertable's kinematics and constrained neural solver.
% Starts trajectory tracking immediately; the scene feature is already centered.
% Retains reference feedforward, gains, constraints and sensing order.
% Shared helpers do not enable learning.
% Defaults: dt = 0.01 s; tracking duration = trajectory.period (20 s).
% Optional config overrides defaultConfig; no online parameter learning is used.
% Four figures show pixel paths, UV errors, nine joint angles (rad),
% and XYZ RCM errors (m).
% Results are saved to Experiment/visual_IBVS_baseline.mat.
% RCM errors are model-derived; computationTime measures only the neural solver.

% Length mismatch example (scene stays at 0.049 m):
%   config.model.flexLen = 0.0392; % m, controller underestimates by 20%
%   See docs/FixedTrajectory_length_and_plotting.md for matched comparisons.
% Both entries use the shared plotFixedTrajectory plotter.

if nargin < 1 || isempty(config)
    config = struct();
end

entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
addpath(rootDir);
commonDir = fullfile(rootDir, 'common');
if ~isfolder(commonDir)
    error('Visual_IBVS_Baseline:MissingCommonDirectory', ...
        'Common function directory was not found: %s', commonDir);
end
addpath(commonDir);
defaults = defaultConfig();
defaults.sim.timeStep = 0.01;
defaults.output.saveFile = fullfile(entryDir, 'visual_IBVS_baseline.mat');
cfg = localMergeStruct(defaults, config);
cfg = resolveModelGeometry(cfg);
if ~isfield(config, 'sim') || ~isfield(config.sim, 'totalTime')
    cfg.sim.totalTime = cfg.sim.initialTime + cfg.trajectory.period;
end
validateattributes(cfg.sim.timeStep, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'positive'});
validateattributes(cfg.sim.totalTime, {'numeric'}, ...
    {'scalar', 'real', 'finite', '>', cfg.sim.initialTime});
assert(cfg.rcm.enabled, 'Visual_IBVS_Baseline requires RCM constraints enabled.');
referenceTrajectory(0, cfg); % Validate before connecting to the scene.
addpath(genpath(fullfile(rootDir, 'vrep_utils')));

numJoints = cfg.robot.numJoints;
n = numJoints + 3; % Seven robot angles, phi, theta, and insertion ratio beta.
visualScale = cfg.control.visualScale;

qDH = cfg.robot.q0(:) + cfg.robot.jointOffset(:);
phi = cfg.robot.phi0;
theta = cfg.robot.theta0;
beta = cfg.rcm.beta0;
z = [qDH; phi; theta; beta];

% Fixed model reference in world coordinates (m), not read from the scene.
d8Trocar = beta * cfg.geometry.shaftLen;
Htrocar = robotKinM(qDH, d8Trocar);
ptrocar = Htrocar(1:3, 4);

maxSteps = ceil((cfg.sim.totalTime - cfg.sim.initialTime) / cfg.sim.timeStep) + 2;
imagData = nan(maxSteps, 2);
imageErrorData = nan(maxSteps, 2);
desiredData = nan(maxSteps, 2);
desiredVelocityData = nan(maxSteps, 2);
pixelErrorData = nan(maxSteps, 2);
jointAngle = nan(maxSteps, n);
jointVelocity = nan(maxSteps, n);
t = nan(maxSteps, 1);
rcmData = nan(maxSteps, 3);
rcmErrorData = nan(maxSteps, 3);
solData = nan(maxSteps, 1);
cmpTime = nan(maxSteps, 1);
numIteration = nan(maxSteps, 1);

mu0 = zeros(5 + n, 1);

simuTime = 0.0;
trajectoryTime = cfg.sim.initialTime;
trackingCount = 0;
motionStepCount = 0;
trackingComplete = false;
noBlobCount = 0;

localDisplay(cfg, sprintf('Continuum length: scene = %.6f m, model = %.6f m (%+.1f%%)', ...
    cfg.geometry.flexLen, cfg.model.flexLen, ...
    100 * (cfg.model.flexLen / cfg.geometry.flexLen - 1)));
localDisplay(cfg, 'Program started: trajectory tracking from the initial scene state');
vrep = [];
clientID = -1;

try
    vrep = remApi('remoteApi');
    vrep.simxFinish(-1);
    clientID = vrep.simxStart(cfg.vrep.address, cfg.vrep.port, ...
        cfg.vrep.waitUntilConnected, cfg.vrep.doNotReconnectOnceDisconnected, ...
        cfg.vrep.timeOutInMs, cfg.vrep.commThreadCycleInMs);

    if clientID <= -1
        error('FixedTrajectoryWithoutCL:VrepConnectionFailed', ...
            'Failed connecting to remote API server.');
    end

    localDisplay(cfg, 'Connected to remote API server');
    vrep.simxSynchronous(clientID, true);

    timeStepCode = vrep.simxSetFloatingParameter(clientID, ...
        vrep.sim_floatparam_simulation_time_step, cfg.sim.timeStep, ...
        vrep.simx_opmode_blocking);
    if timeStepCode ~= vrep.simx_return_ok
        error('FixedTrajectoryWithoutCL:TimeStepConfigurationFailed', ...
            'Failed to set the CoppeliaSim time step to %.4f s.', ...
            cfg.sim.timeStep);
    end

    for jointIdx = 1:numJoints
        motorName = ['Panda_joint' num2str(jointIdx)];
        localGetObjectHandle(vrep, clientID, motorName, cfg, false);
    end

    localGetObjectHandle(vrep, clientID, 'Target', cfg, false);
    cameraHandle = localGetObjectHandle(vrep, clientID, ...
        'blobDetectionCamera_camera', cfg, true);

    vrep.simxReadVisionSensor(clientID, cameraHandle, ...
        vrep.simx_opmode_streaming);

    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);
    pause(cfg.vrep.startPause);

    while vrep.simxGetConnectionId(clientID) ~= -1

        d8 = cfg.geometry.shaftLen ;
        H08 = robotKinM(qDH, d8);
        R08 = H08(1:3, 1:3);
        P08 = H08(1:3, 4);

        H07 = H08 * localTranslateZ(-d8);
        P07 = H07(1:3, 4);

        % Controller kinematics use the assumed length; scene geometry stays separate.
        H09 = H08 * softH([phi; theta], cfg.model.flexLen) * ...
            localTranslateZ(cfg.geometry.cameraLen);
        R09 = H09(1:3, 1:3);
        P09 = H09(1:3, 4);

        Jinst = Jtool([phi; theta], cfg.model.flexLen, cfg.geometry.cameraLen);

        stateData = buildSceneState(qDH, cfg.robot.jointOffset, H08, ...
            phi, theta, cfg);
        returnCode = vrep.simxCallScriptFunction(clientID, 'Panda', ...
            vrep.sim_scripttype_childscript, 'updateState_function', [], ...
            stateData, [], [], vrep.simx_opmode_blocking);

        if returnCode ~= vrep.simx_return_ok
            error('FixedTrajectoryWithoutCL:SceneUpdateFailed', ...
                'Failed to call updateState_function at t = %.4f.', simuTime);
        end

        % Render/sense the newly commanded state before using its pixel sample.
        vrep.simxSynchronousTrigger(clientID);
        vrep.simxGetPingTime(clientID);
        pause(cfg.vrep.stepPause);

        [visionCode, ~, auxData, ~] = vrep.simxReadVisionSensor(clientID, cameraHandle, ...
            vrep.simx_opmode_buffer);
        blobData = double(auxData);

        if visionCode ~= vrep.simx_return_ok || numel(blobData) < 8 || ...
                ~all(isfinite(blobData(5:6)))
            % Match the CL run: do not silently skip feedback after motion.
            if motionStepCount > 0
                error('FixedTrajectoryWithoutCL:LostVision', ...
                    'Valid pixel feedback lost at t = %.4f s.', simuTime);
            end
            noBlobCount = noBlobCount + 1;
            if noBlobCount > cfg.vrep.maxNoBlobSteps
                error('FixedTrajectoryWithoutCL:NoBlobData', ...
                    'Vision sensor did not return valid blob data.');
            end
            pause(cfg.vrep.noBlobPause);
            continue;
        end
        noBlobCount = 0;

        u = cfg.camera.resolutionX - blobData(5) * cfg.camera.resolutionX;
        v = blobData(6) * cfg.camera.resolutionY;
        yScaled = [u; v] * visualScale;
        elapsedTime = trajectoryTime - cfg.sim.initialTime;
        [desiredPixels, desiredPixelVelocity] = ...
            referenceTrajectory(elapsedTime, cfg);
        yTarget = visualScale * desiredPixels;
        eVisual = yScaled - yTarget;

        Jimg = Jimage(u, v, cfg.camera.u0, cfg.camera.v0, cfg.camera.f, ...
            cfg.camera.rho, cfg.camera.Z);

        [J7, Jr] = JrobotM(qDH, d8);
        J8 = Jr(1:3, :);

        Sp = [hat(P08 - P09); zeros(3, 3)];
        Rp = [R08, zeros(3, 3); zeros(3, 3), R08];
        Jsystem = [Jr + Sp * Jr(4:6, :), Rp * Jinst];

        Jtotal = Jimg * [R09.', zeros(3, 3); zeros(3, 3), R09.'] * Jsystem;
        Jv0 = [Jtotal * visualScale, zeros(size(Jtotal, 1), 1)];

        prcm = P07 + beta * (P08 - P07);
        Jrcm = [J7 + beta * (J8 - J7), zeros(3, 2), P08 - P07];

        % Original nominal visual model: no learned Jacobian correction.
        Jtask = [Jv0; Jrcm]; % 5 x 10: two pixel rows and three RCM rows.

        % Moving-reference feedforward plus position-error feedback.
        visualCommand = visualScale * desiredPixelVelocity + ...
            cfg.control.Kimg * (yTarget - yScaled);
        rcmCommand = double(cfg.rcm.enabled) * cfg.rcm.gain * (ptrocar - prcm);
        bTask = [visualCommand; rcmCommand];

        solverTimer = tic;
        [dq, mu0, solverInfo] = neuralVelocitySolver(Jtask, bTask, z, ...
            cfg.limits.qMin, cfg.limits.qMax, cfg.limits.dqMin, ...
            cfg.limits.dqMax, mu0, cfg);
        solverTime = toc(solverTimer);

        if ~all(isfinite(dq)) || ~all(isfinite(mu0)) || ...
                ~isfinite(solverInfo.residual)
            error('FixedTrajectoryWithoutCL:NonfiniteSolverOutput', ...
                'Nonfinite solver output at t = %.4f s.', simuTime);
        end

        pixelError = [u; v] - desiredPixels;
        angleState = [qDH - cfg.robot.jointOffset; phi; theta; beta];
        rcmError = prcm - ptrocar;

        trackingCount = trackingCount + 1;
        if trackingCount > maxSteps
            error('FixedTrajectoryWithoutCL:TrackingBufferExceeded', ...
                'Tracking exceeded its allocated sample count.');
        end
        imagData(trackingCount, :) = [u, v];
        desiredData(trackingCount, :) = desiredPixels.';
        desiredVelocityData(trackingCount, :) = desiredPixelVelocity.';
        pixelErrorData(trackingCount, :) = pixelError.';
        imageErrorData(trackingCount, :) = eVisual.';
        jointAngle(trackingCount, :) = angleState.';
        jointVelocity(trackingCount, :) = dq.';
        t(trackingCount) = trajectoryTime;
        rcmData(trackingCount, :) = prcm.';
        rcmErrorData(trackingCount, :) = rcmError.';
        solData(trackingCount) = solverInfo.residual;
        cmpTime(trackingCount) = solverTime;
        numIteration(trackingCount) = solverInfo.iterations;

        if cfg.output.verbose
            phaseText = ['tracking t = ' num2str(trajectoryTime, '%.3f')];
            disp([phaseText ...
                ', pixel error = ' num2str(norm(pixelError), '%.2f') ...
                ', solver residual = ' num2str(solverInfo.residual, '%.3e')]);
        end

        qDH = qDH + dq(1:7) * cfg.sim.timeStep;
        phi = phi + dq(8) * cfg.sim.timeStep;
        theta = theta + dq(9) * cfg.sim.timeStep;
        beta = beta + dq(10) * cfg.sim.timeStep;
        z = [qDH; phi; theta; beta];

        motionStepCount = motionStepCount + 1;
        simuTime = motionStepCount * cfg.sim.timeStep;

        if trajectoryTime >= cfg.sim.totalTime - cfg.sim.timeStep/2
            trackingComplete = true;
            break;
        end
        trajectoryTime = cfg.sim.initialTime + ...
            trackingCount * cfg.sim.timeStep;
    end

    vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot);
    vrep.simxGetPingTime(clientID);
    vrep.simxFinish(clientID);
    clientID = -1;
    vrep.delete();
    vrep = [];
    localDisplay(cfg, 'Connection is closed');

catch ME
    localCleanupVrep(vrep, clientID);
    rethrow(ME);
end

sampleCount = trackingCount;
idx = 1:sampleCount;

results.config = cfg; % Resolved scene and model lengths, in m.
results.flexLengthRelativeError = ...
    (cfg.model.flexLen - cfg.geometry.flexLen) / cfg.geometry.flexLen;
results.method = 'Nominal-model IBVS without parameter learning';
results.learningEnabled = false;
results.t = t(idx);
results.imagData = imagData(idx, :);
results.imageError = imageErrorData(idx, :); % scaled error used by the controller
results.desiredPixels = desiredData(idx, :);
results.desiredPixelVelocity = desiredVelocityData(idx, :);
results.pixelError = pixelErrorData(idx, :); % actual minus desired, pixels
results.angleRad = jointAngle(idx, 1:9);
results.beta = jointAngle(idx, 10);
results.betaVelocity = jointVelocity(idx, 10); % 1/s, not rad/s
results.rcmSource = 'Forward kinematics of commanded states, not scene measurement';
results.completed = trackingComplete;
results.jointAngle = jointAngle(idx, :);
results.jointVelocity = jointVelocity(idx, :);
results.ptrocar = ptrocar;
results.rcmData = rcmData(idx, :);
results.rcmError = rcmErrorData(idx, :);
results.solverResidual = solData(idx);
results.computationTime = cmpTime(idx); % Neural solver only, in seconds
results.numIteration = numIteration(idx);
results.totalSimulationTime = simuTime;

saveFile = localResolveSaveFile(cfg.output.saveFile, entryDir);
if ~results.completed
    warning('FixedTrajectoryWithoutCL:IncompleteRun', 'Saving an incomplete trajectory run.');
end
imagData = results.imagData;
imageError = results.imageError;
jointAngle = results.jointAngle;
jointVelocity = results.jointVelocity;
t = results.t;
ptrocar = results.ptrocar;
rcmData = results.rcmData;
rcmError = results.rcmError;
solverResidual = results.solverResidual;
computationTime = results.computationTime;
numIteration = results.numIteration;
desiredPixels = results.desiredPixels;
desiredPixelVelocity = results.desiredPixelVelocity;
pixelError = results.pixelError;
save(saveFile, 'results', 'cfg', 'imagData', 'imageError', 'jointAngle', ...
    'jointVelocity', 't', 'ptrocar', 'rcmData', 'rcmError', ...
    'solverResidual', 'computationTime', 'numIteration', ...
    'desiredPixels', 'desiredPixelVelocity', 'pixelError');

if cfg.output.plot && ~isempty(results.t)
    plotFixedTrajectory(results, cfg, saveFile);
end

localDisplay(cfg, ['Task finished. Results saved to ' saveFile]);

end

function H = localTranslateZ(distance)

H = [1 0 0 0; ...
     0 1 0 0; ...
     0 0 1 distance; ...
     0 0 0 1];

end

function handle = localGetObjectHandle(vrep, clientID, objectName, cfg, required)

[returnCode, handle] = vrep.simxGetObjectHandle(clientID, objectName, ...
    vrep.simx_opmode_blocking);

if returnCode ~= vrep.simx_return_ok
    message = ['Handle of ' objectName ' is not found.'];
    if required
        error('FixedTrajectoryWithoutCL:MissingObjectHandle', message);
    else
        warning('FixedTrajectoryWithoutCL:MissingObjectHandle', message);
    end
else
    localDisplay(cfg, ['Find object ' objectName]);
end

end

function cfg = localMergeStruct(defaultCfg, overrideCfg)

cfg = defaultCfg;
if isempty(overrideCfg)
    return;
end
if ~isstruct(overrideCfg)
    error('FixedTrajectoryWithoutCL:InvalidConfig', ...
        'config must be a struct.');
end

fields = fieldnames(overrideCfg);
for k = 1:numel(fields)
    name = fields{k};
    if isfield(cfg, name) && isstruct(cfg.(name)) && isstruct(overrideCfg.(name))
        cfg.(name) = localMergeStruct(cfg.(name), overrideCfg.(name));
    else
        cfg.(name) = overrideCfg.(name);
    end
end

end

function saveFile = localResolveSaveFile(saveFile, rootDir)

if isempty(saveFile)
    saveFile = 'visual_IBVS_baseline.mat';
end

if isstring(saveFile)
    saveFile = char(saveFile);
end

isAbsolute = startsWith(saveFile, filesep) || ...
    ~isempty(regexp(saveFile, '^[A-Za-z]:[\\/]', 'once'));
if ~isAbsolute
    saveFile = fullfile(rootDir, saveFile);
end

end

function localDisplay(cfg, message)

if isfield(cfg, 'output') && isfield(cfg.output, 'verbose') && cfg.output.verbose
    disp(message);
end

end

function localCleanupVrep(vrep, clientID)

try
    if ~isempty(vrep) && clientID > -1
        vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot);
        vrep.simxFinish(clientID);
    end
catch
end

try
    if ~isempty(vrep)
        vrep.delete();
    end
catch
end

end
