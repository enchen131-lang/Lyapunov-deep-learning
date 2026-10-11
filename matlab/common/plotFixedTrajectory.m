function figures = plotFixedTrajectory(results, cfg, saveFile)
%CLIBVS_PLOTFIXEDTRAJECTORY Reference curve styles with existing plot layout.
% Axis limits follow the configured camera; time plots retain seconds.
% saveFile is retained for call compatibility; this function does not export.

colors = [0.000, 0.447, 0.741; 0.850, 0.325, 0.098; ...
    0.929, 0.694, 0.125; 0.494, 0.184, 0.556; ...
    0.466, 0.674, 0.188; 0.301, 0.745, 0.933; ...
    0.635, 0.078, 0.184];
lineTypes = {'-', '--', '-.'};
figures = gobjects(4, 1);

[figures(1), ax] = localAxes('Pixel trajectory tracking');
p2 = plot(ax, results.imagData(:, 1), results.imagData(:, 2), ...
    '-', 'Color', colors(2, :), 'LineWidth', 3);
p1 = plot(ax, results.desiredPixels(:, 1), results.desiredPixels(:, 2), ...
    '--', 'Color', 'k', 'LineWidth', 3);
xlim(ax, [0 cfg.camera.resolutionX]);
ylim(ax, [0 cfg.camera.resolutionY]);
set(ax, 'XDir', 'normal', 'YDir', 'reverse');
localLabels(ax, '$$u$$ (pixel)', '$$v$$ (pixel)');
set(ax.YLabel, 'Position', [0.06, 0.99, 0]);
lgd = localLegend(ax, [p1, p2], {'Desired', 'Measured'});
set(lgd, 'Location', 'southeast');

[figures(2), ax] = localAxes('Pixel tracking errors');
p1 = plot(ax, results.t, results.pixelError(:, 1), ...
    '-', 'Color', colors(1, :), 'LineWidth', 3);
p2 = plot(ax, results.t, results.pixelError(:, 2), ...
    '--', 'Color', colors(2, :), 'LineWidth', 3);
yline(ax, 0, 'k:', 'LineWidth', 1.2, 'HandleVisibility', 'off');
localLabels(ax, '$$t$$ (s)', '$$e$$ (pixel)');
localLegend(ax, [p1, p2], {'u error', 'v error'});

[figures(3), ax] = localAxes('Joint angles');
angleLines = gobjects(9, 1);
angleMarkers = {'+', '^', 'x', 'o', 's', 'd', '*', 'none', 'none'};
angleStyles = {'-', '-', '-', '-', '-', '-', '-', '-.', '--'};
sampleCount = numel(results.t);
markerIndices = unique(round(linspace(1, sampleCount, min(14, sampleCount))));
for k = 1:9
    angleLines(k) = plot(ax, results.t, results.angleRad(:, k), ...
        'Color', colors(mod(k - 1, 7) + 1, :), ...
        'LineStyle', angleStyles{k}, 'Marker', angleMarkers{k}, ...
        'MarkerIndices', markerIndices, 'MarkerSize', 8, ...
        'MarkerFaceColor', 'none', ...
        'LineWidth', 3);
end
localLabels(ax, '$$t$$ (s)', 'Angle (rad)');
angleLimits = ylim(ax);
ylim(ax, [angleLimits(1), angleLimits(2) + 0.15 * diff(angleLimits)]);
set(ax.XLabel, 'Position', [0.45, 0.15, 0]);
lgd = localLegend(ax, angleLines, ...
    {'q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6', 'q_7', '\phi', '\theta'});
% Back2Center also reduces dense, multi-curve legends from 24 to 18 pt.
set(lgd, 'NumColumns', 3, 'FontSize', 18, 'Location', 'southeast');

[figures(4), ax] = localAxes('RCM position errors (model)');
rcmLines = gobjects(3, 1);
for k = 1:3
    rcmLines(k) = plot(ax, results.t, results.rcmError(:, k), ...
        'Color', colors(k, :), 'LineStyle', lineTypes{k}, 'LineWidth', 3);
end
yline(ax, 0, 'k:', 'LineWidth', 1.2, 'HandleVisibility', 'off');
localLabels(ax, '$$t$$ (s)', '$$e_{\mathrm{RCM}}$$ (m)');
localLegend(ax, rcmLines, {'x error', 'y error', 'z error'});

end

function [fig, ax] = localAxes(name)

fig = figure('Color', 'w', 'Name', name, 'Position', [100, 100, 800, 500]);
% Match the axes rectangle to 1.25:1 so legends stay inside the plot box.
ax = axes('Parent', fig, 'Units', 'pixels', 'Position', [130, 60, 487.5, 390]);
set(ax, 'Units', 'normalized');
hold(ax, 'on');
grid(ax, 'off');
box(ax, 'on');
set(ax, 'FontName', 'Times New Roman', 'FontSize', 24, ...
    'LineWidth', 1.2, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', ...
    'PlotBoxAspectRatio', [1.25, 1, 1]);

end

function localLabels(ax, xText, yText)

xx = xlabel(ax, xText, 'Interpreter', 'latex', ...
    'FontName', 'Times New Roman', 'FontSize', 24);
yy = ylabel(ax, yText, 'Interpreter', 'latex', ...
    'FontName', 'Times New Roman', 'FontSize', 24);
set(yy, 'Rotation', 0, 'Units', 'normalized', ...
    'HorizontalAlignment', 'left', 'VerticalAlignment', 'top', ...
    'Position', [0.06, 0.96, 0]);
set(xx, 'Units', 'normalized', 'HorizontalAlignment', 'right', ...
    'VerticalAlignment', 'bottom', 'Position', [0.55, 0.15, 0]);

end

function lgd = localLegend(ax, lines, labels)

lgd = legend(ax, lines, labels, 'FontName', 'Times New Roman', ...
    'FontSize', 24, 'NumColumns', 1, 'Location', 'northeast');

end
