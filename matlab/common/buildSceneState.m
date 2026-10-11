function stateData = buildSceneState(qDH, jointOffset, H08, phi, theta, cfg)
%CLIBVS_BUILDSCENESTATE Scene geometry only; never use cfg.model.flexLen here.
% Payload: seven joints, nine disk transforms, one camera transform (127 x 1).

Hcamera = H08 * softH([phi; theta], cfg.geometry.flexLen) * ...
    localTranslateZ(cfg.geometry.cameraLen) * ...
    localTranslateZ(-cfg.geometry.cameraLen/2);
cameraData = reshape(Hcamera(1:3, :), 12, 1);

diskLengths = cfg.geometry.diskLengths(:);
diskData = zeros(12 * numel(diskLengths), 1);
for k = 1:numel(diskLengths)
    ratio = diskLengths(k) / cfg.geometry.flexLen;
    Hdisk = H08 * softH([phi; ratio * theta], ratio * cfg.geometry.flexLen);
    diskData((k - 1)*12 + (1:12)) = reshape(Hdisk(1:3, :), 12, 1);
end
stateData = [qDH(:) - jointOffset(:); diskData; cameraData];

end

function H = localTranslateZ(distance)

H = eye(4);
H(3, 4) = distance;

end
