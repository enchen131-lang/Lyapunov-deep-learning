function [desiredPixels, desiredVelocity] = referenceTrajectory(time, cfg)
%CLIBVS_REFERENCETRAJECTORY Pixel path and analytic velocity, both 2 x N.
% time is elapsed time in seconds; velocity is in pixels/s.
% Uses the executable v formula in Reference/trajectory.m, not its header.

if nargin < 2 || isempty(cfg)
    cfg = defaultConfig();
end
validateattributes(time, {'numeric'}, {'real', 'finite', 'vector', 'nonempty'});
validateattributes(cfg.trajectory.amplitude, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'});
validateattributes(cfg.trajectory.period, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'positive'});
validateattributes(cfg.trajectory.center, {'numeric'}, ...
    {'real', 'finite', 'numel', 2});

phase = (2*pi/cfg.trajectory.period) * time(:).';
amplitude = cfg.trajectory.amplitude;
omega = 2*pi/cfg.trajectory.period;
center = cfg.trajectory.center(:);
desiredPixels = [center(1) + amplitude*sin(2*phase).*cos(phase); ...
    center(2) + amplitude*sin(2*phase).*sin(phase)];
desiredVelocity = amplitude*omega * ...
    [2*cos(2*phase).*cos(phase) - sin(2*phase).*sin(phase); ...
     2*cos(2*phase).*sin(phase) + sin(2*phase).*cos(phase)];
end
