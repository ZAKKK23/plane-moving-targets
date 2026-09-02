%% ============================================================
%  dynamics.m  —  VTP STADIUM TRACK (closed-loop moving targets)
%
%  Same VTP agent dynamics as always (repulsion + alignment + homing
%  over the Delaunay neighbor graph — see neighborhoods.m, alignTo.m,
%  transition.m, voronoiProjectToBoundary.m, Target.m, all unchanged).
%  What's different here is the target path: a closed "stadium" loop
%  made of two straight lines and two half-circles (like a running
%  track) — you pick how many targets ride the loop, how far apart
%  they're spaced along it, and the loop's own shape.
%
%  Track parametrization (arc length s, measured from the bottom of
%  the right-hand straight, increasing counter-clockwise):
%    - s in [0, straightLen):                          right straight, going up
%    - s in [straightLen, straightLen+pi*R):            top half-circle
%    - s in [straightLen+pi*R, 2*straightLen+pi*R):      left straight, going down
%    - s in [2*straightLen+pi*R, 2*straightLen+2*pi*R):  bottom half-circle
%  Total perimeter = 2*straightLen + 2*pi*R. All targets share one
%  arc-length accumulator, offset by a constant spacing, so their
%  along-track spacing is exact and never drifts (same idea as the
%  earlier aligned/corridor versions, just wrapped onto a closed loop).
%
%  A control panel lets you change, live:
%    - Cells (agents') overall speed
%    - nu (alignment strength) and L (interaction length scale)
%    - Straight length and corner radius of the track itself
%    - Spacing between targets along the track, and their shared speed
%    - Apply the above live, Pause/Resume, and Reset targets
%
%  IMPLEMENTATION NOTE: All button callbacks only set a flag via
%  setappdata on the figure; the flags are read back inside the main
%  loop below. This avoids relying on MATLAB's local-function variable
%  scoping in script files, which is not supported consistently across
%  MATLAB versions/configurations. stadiumPos (defined at the end of
%  this file) is a plain, argument-only local function — safe to call
%  directly, no closures involved.
%% ============================================================

%% ---- Simulation parameters ----------------------------------
N      = 200;
L      = 1;         % interaction length scale — editable live (see cur_L below)
nu     = 2.5;        % alignment strength — editable live (see cur_nu below)
tmax   = 5000;

cell_spd0 = 1;      % default cell speed multiplier

straightLen0   = 10;   % default length of each straight side of the track
cornerRadius0  = 5;     % default radius of each half-circle end cap
targetSpacing0 = 8;     % default arc-length spacing between targets along the track
trackSpeed0    = 0.15;  % default arc-length speed of every target along the track

%% ---- Ask user for number of targets ---------------------------
try
    nT_answer = inputdlg('Number of targets on the loop (1-10):', 'Setup', 1, {'4'});
catch
    nT_answer = {};
end
if isempty(nT_answer)
    nT = 4;   % default if dialog cancelled or unavailable
else
    nT = round(str2double(nT_answer{1}));
    if isnan(nT), nT = 4; end
end
nT = max(1, min(10, nT));

%% ---- Initial agent conditions -------------------------------
ic_rad = 0.5 * sqrt(N*pi/4/0.91);
rng(2);
X   = ic_rad * (2*rand(N,2) - 1);
rng(18);
ang = 2*pi * rand(N,1);
U   = [cos(ang) sin(ang)];

%% ---- Initial target state --------------------------------------
sAccum = 0;   % shared arc-length accumulator — every target's position
              % is sAccum + (k-1)*targetSpacing, wrapped onto the track

Tpos = zeros(nT, 2);
for k = 1:nT
    Tpos(k,:) = stadiumPos((k-1)*targetSpacing0, straightLen0, cornerRadius0);
end

%% ---- Build initial Target object ----------------------------
tar = Target( mat2cell(Tpos, ones(nT,1), 2) );

%% ---- Preallocate agent variables ----------------------------
U1 = zeros(N,2);

%% ============================================================
%  Build figure + UI
%% ============================================================
colors = lines(nT);

PANEL_H  = 0.32;   % fixed — no per-target rows needed in this version
fig = figure('Name', 'VTP - Stadium Track', ...
             'NumberTitle', 'off', ...
             'Position', [80 40 980 860]);

ax = axes('Parent', fig, ...
          'Position', [0.05  PANEL_H+0.03  0.90  0.94-PANEL_H]);

pan = uipanel('Parent', fig, ...
              'Title', 'Track Controls', ...
              'Position', [0.01 0.005 0.98 PANEL_H], ...
              'FontSize', 9);

% ---- Cells (agents) speed row ---------------------------------
uicontrol(pan, 'Style','text', ...
    'String', 'Cells', ...
    'FontWeight','bold', 'FontSize', 9, ...
    'Units','normalized', ...
    'Position', [0.01  0.80  0.06  0.16]);

uicontrol(pan,'Style','text','String','Spd:', ...
    'Units','normalized','Position',[0.07 0.80 0.04 0.16],...
    'FontSize',8,'HorizontalAlignment','right');

sld_cell = uicontrol(pan,'Style','slider', ...
    'Min',0,'Max',3,'Value', cell_spd0, ...
    'Units','normalized','Position',[0.12  0.82  0.30  0.11]);

lbl_cell = uicontrol(pan,'Style','text', ...
    'String', sprintf('%.2f', cell_spd0), ...
    'Units','normalized','Position',[0.43 0.80 0.07 0.16],...
    'FontSize',8,'HorizontalAlignment','left');

addlistener(sld_cell,'Value','PostSet', ...
    @(~,~) set(lbl_cell,'String', sprintf('%.2f', sld_cell.Value)));

% ---- Dynamics constants (nu, L) row ----------------------------
uicontrol(pan, 'Style','text', ...
    'String', 'Dynamics', ...
    'FontWeight','bold', 'FontSize', 9, ...
    'Units','normalized', ...
    'Position', [0.01  0.61  0.09  0.16]);

uicontrol(pan,'Style','text','String','nu:', ...
    'Units','normalized','Position',[0.11 0.61 0.05 0.16],...
    'FontSize',8,'HorizontalAlignment','right');

edt_nu = uicontrol(pan,'Style','edit', ...
    'String', num2str(nu), ...
    'Units','normalized','Position',[0.17  0.635  0.13  0.115], ...
    'FontSize',8, 'BackgroundColor', [1 1 1]);

uicontrol(pan,'Style','text','String','L:', ...
    'Units','normalized','Position',[0.35 0.61 0.04 0.16],...
    'FontSize',8,'HorizontalAlignment','right');

edt_L = uicontrol(pan,'Style','edit', ...
    'String', num2str(L), ...
    'Units','normalized','Position',[0.40  0.635  0.13  0.115], ...
    'FontSize',8, 'BackgroundColor', [1 1 1]);

% ---- Path row: straight length, corner radius --------------------
uicontrol(pan, 'Style','text', ...
    'String', 'Path', ...
    'FontWeight','bold', 'FontSize', 9, ...
    'Units','normalized', ...
    'Position', [0.01  0.40  0.07  0.16]);

uicontrol(pan,'Style','text','String','Straight len:', ...
    'Units','normalized','Position',[0.09 0.40 0.11 0.16],...
    'FontSize',8,'HorizontalAlignment','right');
sld_straight = uicontrol(pan,'Style','slider', ...
    'Min',0,'Max',30,'Value', straightLen0, ...
    'Units','normalized','Position',[0.21  0.42  0.20  0.11]);
lbl_straight = uicontrol(pan,'Style','text', ...
    'String', sprintf('%.2f', straightLen0), ...
    'Units','normalized','Position',[0.42 0.40 0.06 0.16],...
    'FontSize',8,'HorizontalAlignment','left');

uicontrol(pan,'Style','text','String','Corner radius:', ...
    'Units','normalized','Position',[0.50 0.40 0.12 0.16],...
    'FontSize',8,'HorizontalAlignment','right');
sld_radius = uicontrol(pan,'Style','slider', ...
    'Min',0,'Max',20,'Value', cornerRadius0, ...
    'Units','normalized','Position',[0.63  0.42  0.20  0.11]);
lbl_radius = uicontrol(pan,'Style','text', ...
    'String', sprintf('%.2f', cornerRadius0), ...
    'Units','normalized','Position',[0.84 0.40 0.06 0.16],...
    'FontSize',8,'HorizontalAlignment','left');

addlistener(sld_straight,'Value','PostSet', ...
    @(~,~) set(lbl_straight,'String', sprintf('%.2f', sld_straight.Value)));
addlistener(sld_radius,'Value','PostSet', ...
    @(~,~) set(lbl_radius,'String', sprintf('%.2f', sld_radius.Value)));

% ---- Targets row: spacing, speed ---------------------------------
uicontrol(pan, 'Style','text', ...
    'String', 'Targets', ...
    'FontWeight','bold', 'FontSize', 9, ...
    'Units','normalized', ...
    'Position', [0.01  0.20  0.08  0.16]);

uicontrol(pan,'Style','text','String','Spacing:', ...
    'Units','normalized','Position',[0.10 0.20 0.08 0.16],...
    'FontSize',8,'HorizontalAlignment','right');
sld_spacing = uicontrol(pan,'Style','slider', ...
    'Min',0,'Max',30,'Value', targetSpacing0, ...
    'Units','normalized','Position',[0.19  0.22  0.20  0.11]);
lbl_spacing = uicontrol(pan,'Style','text', ...
    'String', sprintf('%.2f', targetSpacing0), ...
    'Units','normalized','Position',[0.40 0.20 0.06 0.16],...
    'FontSize',8,'HorizontalAlignment','left');

uicontrol(pan,'Style','text','String','Speed:', ...
    'Units','normalized','Position',[0.48 0.20 0.06 0.16],...
    'FontSize',8,'HorizontalAlignment','right');
sld_speed = uicontrol(pan,'Style','slider', ...
    'Min',0,'Max',1,'Value', trackSpeed0, ...
    'Units','normalized','Position',[0.55  0.22  0.20  0.11]);
lbl_speed = uicontrol(pan,'Style','text', ...
    'String', sprintf('%.3f', trackSpeed0), ...
    'Units','normalized','Position',[0.76 0.20 0.07 0.16],...
    'FontSize',8,'HorizontalAlignment','left');

addlistener(sld_spacing,'Value','PostSet', ...
    @(~,~) set(lbl_spacing,'String', sprintf('%.2f', sld_spacing.Value)));
addlistener(sld_speed,'Value','PostSet', ...
    @(~,~) set(lbl_speed,'String', sprintf('%.3f', sld_speed.Value)));

% ---- Buttons at the bottom of the panel ----------------------
setappdata(fig, 'apply_clicked', false);
setappdata(fig, 'reset_clicked', false);

btn_apply = uicontrol(pan,'Style','pushbutton','String','Apply', ...
    'FontSize',9,'FontWeight','bold', ...
    'Units','normalized','Position',[0.01 0.02 0.12 0.15], ...
    'Callback', @(src,evt) setappdata(fig,'apply_clicked',true));

btn_pause = uicontrol(pan,'Style','togglebutton','String','Pause', ...
    'FontSize',9, ...
    'Units','normalized','Position',[0.15 0.02 0.12 0.15]);

btn_reset = uicontrol(pan,'Style','pushbutton','String','Reset Targets', ...
    'FontSize',9, ...
    'Units','normalized','Position',[0.29 0.02 0.16 0.15], ...
    'Callback', @(src,evt) setappdata(fig,'reset_clicked',true));

%% ---- Mutable simulation variables (plain script variables) --
cur_cell_spd      = cell_spd0;
cur_nu            = nu;
cur_L             = L;
cur_straightLen   = straightLen0;
cur_cornerRadius  = cornerRadius0;
cur_targetSpacing = targetSpacing0;
cur_trackSpeed    = trackSpeed0;

%% ============================================================
%  Main simulation loop
%% ============================================================
t = 0;
while t < tmax && ishandle(fig)

    % Pause
    while ishandle(fig) && get(btn_pause,'Value')
        pause(0.05);
    end
    if ~ishandle(fig), break; end

    % ---- Check Apply button ----
    if getappdata(fig, 'apply_clicked')
        setappdata(fig, 'apply_clicked', false);
        cur_cell_spd      = get(sld_cell, 'Value');
        cur_straightLen   = get(sld_straight, 'Value');
        cur_cornerRadius  = get(sld_radius, 'Value');
        cur_targetSpacing = get(sld_spacing, 'Value');
        cur_trackSpeed    = get(sld_speed, 'Value');

        newNu = str2double(get(edt_nu, 'String'));
        if isnan(newNu) || newNu < 0
            newNu = cur_nu;
        end
        cur_nu = newNu;
        set(edt_nu, 'String', num2str(cur_nu));

        newL = str2double(get(edt_L, 'String'));
        if isnan(newL) || newL <= 0
            newL = cur_L;
        end
        cur_L = newL;
        set(edt_L, 'String', num2str(cur_L));
    end

    % ---- Check Reset button ----
    if getappdata(fig, 'reset_clicked')
        setappdata(fig, 'reset_clicked', false);

        sAccum            = 0;
        cur_cell_spd      = cell_spd0;
        cur_straightLen   = straightLen0;
        cur_cornerRadius  = cornerRadius0;
        cur_targetSpacing = targetSpacing0;
        cur_trackSpeed    = trackSpeed0;

        set(sld_cell,'Value', cell_spd0);
        set(lbl_cell,'String', sprintf('%.2f', cell_spd0));
        set(sld_straight,'Value', straightLen0);
        set(lbl_straight,'String', sprintf('%.2f', straightLen0));
        set(sld_radius,'Value', cornerRadius0);
        set(lbl_radius,'String', sprintf('%.2f', cornerRadius0));
        set(sld_spacing,'Value', targetSpacing0);
        set(lbl_spacing,'String', sprintf('%.2f', targetSpacing0));
        set(sld_speed,'Value', trackSpeed0);
        set(lbl_speed,'String', sprintf('%.3f', trackSpeed0));
        t = 0;
    end

    t = t + 1;

    %% -- Move targets along the stadium track -------------------
    % Every target shares the SAME arc-length accumulator, offset by a
    % constant spacing, so their along-track spacing is exact and never
    % drifts — even while spacing/speed/shape are being changed live.
    sAccum = sAccum + cur_trackSpeed;
    for k = 1:nT
        Tpos(k,:) = stadiumPos(sAccum + (k-1)*cur_targetSpacing, cur_straightLen, cur_cornerRadius);
    end

    tar = Target( mat2cell(Tpos, ones(nT,1), 2) );

    %% -- Agent dynamics ---------------------------------------
    DT               = delaunayTriangulation(X);
    [nbhd, nearest, d] = neighborhoods(DT);
    fun              = @(x) transition(x, 'expReciprocal');
    s                = arrayfun(fun, d/cur_L);

    % repulsion
    r_rep  = X - X(nearest,:);
    rnorm  = vecnorm(r_rep, 2, 2);
    rnorm(rnorm < eps) = eps;   % avoid divide-by-zero if agents coincide
    r_rep  = s .* r_rep ./ rnorm;

    % alignment
    a_ali  = alignTo(U, nbhd, 'expReciprocal');

    % homing
    h0     = homeToTarget(tar, X);
    h      = (1-s) .* h0 ./ vecnorm(h0, 2, 2);
    h(isnan(h)) = 0;

    % direction
    U1     = (r_rep + h + cur_nu*a_ali) / (1 + cur_nu);

    % speed (uses fixed voronoiProjectToBoundary)
    [~, l] = voronoiProjectToBoundary(DT, U1);
    M      = l;

    %% -- Plot -------------------------------------------------
    if ishandle(fig)
        com  = mean(X);
        rmed = sqrt(median((X(:,1)-com(1)).^2 + (X(:,2)-com(2)).^2));
        cx   = com(1);  cy = com(2);  hw = 3*rmed;

        all_pts = [X; Tpos];
        xlo = min(cx-hw, min(all_pts(:,1))-2);
        xhi = max(cx+hw, max(all_pts(:,1))+2);
        ylo = min(cy-hw, min(all_pts(:,2))-2);
        yhi = max(cy+hw, max(all_pts(:,2))+2);
        hw2 = max(xhi-xlo, yhi-ylo)/2;
        cx2 = (xlo+xhi)/2;  cy2 = (ylo+yhi)/2;

        cla(ax);
        set(ax, 'XLim', [cx2-hw2, cx2+hw2], ...
                'YLim', [cy2-hw2, cy2+hw2], ...
                'DataAspectRatio', [1 1 1], ...
                'NextPlot', 'add');

        % draw the track itself, as a faint guide
        sTrack = linspace(0, 2*cur_straightLen + 2*pi*cur_cornerRadius, 200);
        trackPts = zeros(numel(sTrack), 2);
        for kk = 1:numel(sTrack)
            trackPts(kk,:) = stadiumPos(sTrack(kk), cur_straightLen, cur_cornerRadius);
        end
        plot(ax, trackPts(:,1), trackPts(:,2), '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1);

        % Agents
        scatter(ax, X(:,1), X(:,2), 4, 'k', 'filled');
        quiver(ax, X(:,1), X(:,2), U1(:,1), U1(:,2), ...
               'b', 'AutoScaleFactor', 0.5, 'MaxHeadSize', 0.3);

        % Moving targets
        theta_c = linspace(0, 2*pi, 40);
        r_c     = 0.7;
        for k = 1:nT
            fill(ax, Tpos(k,1) + r_c*cos(theta_c), ...
                     Tpos(k,2) + r_c*sin(theta_c), ...
                     colors(k,:), 'EdgeColor','none', 'FaceAlpha', 0.9);
            text(ax, Tpos(k,1), Tpos(k,2), sprintf('T%d', k), ...
                 'Color','w','FontWeight','bold', ...
                 'HorizontalAlignment','center','FontSize',7);
        end

        set(ax, 'NextPlot', 'replace');
        title(ax, sprintf('t = %d    |    %d targets on stadium track    |    straight = %.2f, radius = %.2f, spacing = %.2f, speed = %.3f, \nu = %.2f, L = %.2f', ...
              t, nT, cur_straightLen, cur_cornerRadius, cur_targetSpacing, cur_trackSpeed, cur_nu, cur_L), 'FontSize', 9);
        drawnow limitrate;
    end

    %% -- Update agent state -----------------------------------
    U = cur_cell_spd * tanh(M/cur_L) .* U1;
    X = X + U;

end   % end main loop


%% ============================================================
%  Local functions
%% ============================================================
function pos = stadiumPos(s, straightLen, R)
% STADIUMPOS  Position at arc-length s along a closed "stadium" track:
%   two straight segments of length straightLen, joined by two
%   half-circles of radius R. s is measured from the bottom of the
%   right-hand straight, increasing counter-clockwise, and wraps
%   automatically (mod the total perimeter). Degenerate inputs
%   (straightLen = 0 and/or R = 0) are handled without division by
%   zero: whichever segment has zero length simply contributes zero
%   width to the parametrization.
    perim = 2*straightLen + 2*pi*R;
    s = mod(s, perim);
    if s < straightLen
        pos = [R, -straightLen/2 + s];
    elseif s < straightLen + pi*R
        theta = (s - straightLen) / R;
        pos = [R*cos(theta), straightLen/2 + R*sin(theta)];
    elseif s < 2*straightLen + pi*R
        pos = [-R, straightLen/2 - (s - (straightLen + pi*R))];
    else
        theta = pi + (s - (2*straightLen + pi*R)) / R;
        pos = [R*cos(theta), -straightLen/2 + R*sin(theta)];
    end
end
