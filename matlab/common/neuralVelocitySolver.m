function [dq, muNext, info] = neuralVelocitySolver(Jtask, bTask, z, qMin, qMax, dqMin, dqMax, mu0, cfg)
%CLIBVS_NEURALVELOCITYSOLVER Constrained neural-dynamic velocity solver.
%
%   This function preserves the original equality/inequality projection
%   structure used by Panda_RCM_RNN3_insertable.m.

if nargin < 9 || isempty(cfg)
    cfg = defaultConfig();
end

z = z(:);
qMin = qMin(:);
qMax = qMax(:);
dqMin = dqMin(:);
dqMax = dqMax(:);
bTask = bTask(:);

n = size(Jtask, 2);
m = size(Jtask, 1);
if numel(z) ~= n
    error('neuralVelocitySolver:InvalidStateLength', ...
        'Expected z length %d, got %d.', n, numel(z));
end
if numel(bTask) ~= m
    error('neuralVelocitySolver:InvalidTaskLength', ...
        'Expected bTask length %d, got %d.', m, numel(bTask));
end

validateattributes(cfg.sim.timeStep, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'positive'});
validateattributes(cfg.neural.limitStepFraction, {'numeric'}, ...
    {'scalar', 'real', 'finite', '>', 0, '<=', 1});
validateattributes(cfg.control.phiCenterGain, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'nonnegative'});
validateattributes(cfg.control.phiReference, {'numeric'}, ...
    {'scalar', 'real', 'finite'});

phiIndex = cfg.robot.numJoints + 1;
if phiIndex > n
    error('neuralVelocitySolver:MissingPhiState', ...
        'Expected phi at state index %d, but the task has %d states.', ...
        phiIndex, n);
end

tempJ = [Jtask; eye(n, n)];
if nargin < 8 || isempty(mu0) || numel(mu0) ~= m + n
    mu = zeros(m + n, 1);
else
    mu = mu0(:);
end

positionLimitGain = min(cfg.neural.kxi, ...
    cfg.neural.limitStepFraction / cfg.sim.timeStep);
dLower = max(dqMin, positionLimitGain * (qMin - z));
dUpper = min(dqMax, positionLimitGain * (qMax - z));
zL = [bTask; dLower];
zU = [bTask; dUpper];

M = cfg.neural.M;
if isscalar(M)
    M = M * eye(n);
end

dqReference = zeros(n, 1);
dqReference(phiIndex) = -cfg.control.phiCenterGain * ...
    (z(phiIndex) - cfg.control.phiReference);

dq = dqReference - M * (tempJ.' * mu);
err = inf;
iterations = 0;

for k = 1:cfg.neural.iterNum
    x = -cfg.neural.alpha * tempJ * dq;
    y = gFun(cfg.neural.alpha, mu - x, zL, zU);
    y = y + x;

    mu = mu - cfg.neural.tau * cfg.neural.gamma * ...
        actFun(y, cfg.neural.r, cfg.neural.caseFun);

    dq = dqReference - M * (tempJ.' * mu);
    err = norm(y);
    iterations = k;

    if err < cfg.neural.errTol
        break;
    end
end

dqBeforeProjection = dq;
dq = min(max(dq, dLower), dUpper);

muNext = mu;
info.residual = err;
info.iterations = iterations;
info.lowerVelocityBound = dLower;
info.upperVelocityBound = dUpper;
info.referenceVelocity = dqReference;
info.positionLimitGain = positionLimitGain;
info.velocityProjectionNorm = norm(dq - dqBeforeProjection);
info.taskResidualAfterProjection = norm(Jtask * dq - bTask);

end
