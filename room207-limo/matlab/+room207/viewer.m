function out = viewer(action, varargin)
%VIEWER Room 207 view and keyboard teleoperation for the Simulink models.
%
%   room207.viewer('start', model, names)
%       Open (or reset) the view before a simulation. names is a cell
%       array with one name per robot drawn, e.g. {'Simulink LIMO'}.
%
%   u = room207.viewer('command')
%       Keyboard command u = [v; gamma] (AZERTY, as in the ROS 2 demo):
%           Z / S   forward / backward at max_speed (held)
%           Q / D   steer left / right at max_steering (held)
%           Space   stop
%           Esc     stop the simulation
%       The figure must have the keyboard focus.
%
%   room207.viewer('draw', t, poses)
%       Draw the robots at time t. poses is 3 x N, one [X; Y; psi] column
%       per robot. Columns containing NaN are skipped.
%
%   room207.viewer('stop')
%       Mark the end of the simulation.

persistent S

out = [];

switch action
    case 'start'
        S = start_view(S, varargin{1}, varargin{2});

    case 'command'
        out = keyboard_command(S);

    case 'draw'
        S = draw(S, varargin{1}, varargin{2});

    case 'key'
        S = key_event(S, varargin{1}, varargin{2});

    case 'stop'
        if is_open(S)
            S.status.String = 'Simulation stopped';
        end

    otherwise
        error('room207:viewer', 'Unknown action "%s".', action);
end

end


function S = start_view(S, model, names)
%START_VIEW Create the figure, or reuse it and clear the trajectories.

d = room207.parameters();

if ~is_open(S)
    S = struct();
    S.d = d;

    S.fig = figure( ...
        'Name', 'ROOM 207 | LIMO', ...
        'NumberTitle', 'off', ...
        'Color', 'w', ...
        'WindowKeyPressFcn', @(~, e) room207.viewer('key', 'press', e.Key), ...
        'WindowKeyReleaseFcn', @(~, e) room207.viewer('key', 'release', e.Key), ...
        'CloseRequestFcn', @(f, ~) close_view(f));

    % Light colours whatever the MATLAB theme, as on the printed map
    theme(S.fig, 'light');

    S.ax = axes(S.fig);
    hold(S.ax, 'on');

    w = d.map.width/2;
    h = d.map.height/2;

    if isfile(d.map.file)
        image(S.ax, 'XData', [-w w], 'YData', [h -h], 'CData', imread(d.map.file));
    else
        S.ax.Color = [0.91 0.93 0.95];
        warning('room207:viewer', 'Track map not found: %s', d.map.file);
    end

    axis(S.ax, 'equal');
    S.ax.YDir = 'normal';
    S.ax.XLim = [-w w];
    S.ax.YLim = [-h h];
    xlabel(S.ax, 'x (m)');
    ylabel(S.ax, 'y (m)');

    S.status = title(S.ax, '');
    subtitle(S.ax, 'Click here, then drive with Z Q S D (Space: stop, Esc: end)');
else
    delete(S.trails);
    delete(S.bodies);
    delete(S.legend);
end

colors = [0.90 0.22 0.21; 0.00 0.40 0.60; 0.30 0.60 0.20];
n = numel(names);

S.trails = gobjects(n, 1);
S.bodies = gobjects(n, 1);

for i = 1:n
    c = colors(1 + mod(i-1, size(colors, 1)), :);
    S.trails(i) = plot(S.ax, NaN, NaN, '-', 'Color', c, 'LineWidth', 1.8);
    S.bodies(i) = patch(S.ax, NaN, NaN, c, 'FaceAlpha', 0.6, 'EdgeColor', c);
end

S.legend = legend(S.ax, S.trails, names, 'Location', 'northoutside');

S.model = model;
setappdata(S.fig, 'model', model);
S.pressed = {};
S.status.String = 'Starting...';

drawnow;

end


function u = keyboard_command(S)
%KEYBOARD_COMMAND [v; gamma] from the keys currently held.

u = [0; 0];

if ~is_open(S)
    return
end

held = @(k) any(strcmp(S.pressed, k));

if held('z') && ~held('s')
    u(1) = S.d.teleop.max_speed;
elseif held('s') && ~held('z')
    u(1) = -S.d.teleop.max_speed;
end

if held('q') && ~held('d')
    u(2) = S.d.teleop.max_steering;
elseif held('d') && ~held('q')
    u(2) = -S.d.teleop.max_steering;
end

end


function S = key_event(S, type, key)
%KEY_EVENT Update the held keys.

if ~is_open(S)
    return
end

key = lower(key);

switch type
    case 'press'
        if any(strcmp(key, {'z', 'q', 's', 'd'}))
            S.pressed = union(S.pressed, {key});
        elseif strcmp(key, 'space')
            S.pressed = {};
        elseif strcmp(key, 'escape')
            stop_simulation(S.model);
        end

    case 'release'
        S.pressed = setdiff(S.pressed, {key});
end

end


function S = draw(S, t, poses)
%DRAW Append the poses to the trajectories and move the footprints.

if ~is_open(S)
    return
end

L = S.d.view.footprint(1)/2;
W = S.d.view.footprint(2)/2;

% Footprint with a nose showing the heading
shape = [-L -W; L -W; 1.25*L 0; L W; -L W].';

for i = 1:min(size(poses, 2), numel(S.trails))
    p = poses(:,i);

    if any(isnan(p))
        continue
    end

    S.trails(i).XData(end+1) = p(1);
    S.trails(i).YData(end+1) = p(2);

    R = [cos(p(3)) -sin(p(3)); sin(p(3)) cos(p(3))];
    v = R*shape + p(1:2);
    S.bodies(i).XData = v(1,:);
    S.bodies(i).YData = v(2,:);
end

S.status.String = sprintf('t = %.1f s', t);

drawnow limitrate;

end


function close_view(fig)
%CLOSE_VIEW Closing the view also stops the simulation.

model = getappdata(fig, 'model');

delete(fig);

if ~isempty(model)
    stop_simulation(model);
end

end


function stop_simulation(model)
%STOP_SIMULATION Stop the model if it is running.

if bdIsLoaded(model) && ~strcmp(get_param(model, 'SimulationStatus'), 'stopped')
    set_param(model, 'SimulationCommand', 'stop');
end

end


function tf = is_open(S)
%IS_OPEN True if the view figure exists.

tf = ~isempty(S) && isfield(S, 'fig') && isgraphics(S.fig);

end
