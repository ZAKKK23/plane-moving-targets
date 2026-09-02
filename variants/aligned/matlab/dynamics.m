%% ============================================================
%  dynamics.m  —  VTP plane simulation with ALIGNED MOVING TARGETS
%
%  Same VTP agent dynamics as always (repulsion + alignment + homing
%  over the Delaunay neighbor graph — see neighborhoods.m, alignTo.m,
%  transition.m, voronoiProjectToBoundary.m, Target.m, all unchanged).
%  What's different here is how the TARGETS move:
%
%    - You choose a number of "straight-line" targets (0-5). Each one
%      travels in a perfectly straight horizontal line at a constant
%      height (its own "Height" parameter, independently adjustable) —
%      like several parallel horizontal lanes.
%    - One additional target is always added on top of those: it moves
%      sinusoidally in y (its own Height/Amplitude/Omega parameters)
%      while still moving horizontally at the same constant speed as
%      every other target — so it travels in a wavy, oscillating path.
%    - ALL targets — including the oscillating one — share a single
%      horizontal speed (vx) and start on the same vertical line, so
%      they always have exactly the same x-coordinate at every instant.
%      There is only one shared x(t); each target only differs in y(t).
%
%  A control panel lets you:
%    - Change the cells' (agents') overall speed
%    - Change the two constants that govern the cell dynamics: nu
%      (alignment strength) and L (interaction length scale)
%    - Change the shared horizontal speed (vx) of every target
%    - Change each straight-line target's height
%    - Change the oscillating target's height (oscillation center),
%      amplitude, and omega (angular frequency)
%    - Apply the above live, Pause/Resume, and Reset targets
%
%  IMPLEMENTATION NOTE: All button callbacks only set a flag via
%  setappdata on the figure; the flags are read back inside the main
%  loop below. This avoids relying on MATLAB's local-function variable
%  scoping in script files, which is not supported consistently across
%  MATLAB versions/configurations.
%% ============================================================

%% ---- Simulation parameters ----------------------------------
N      = 200;
L      = 1;         % interaction length scale — editable live (see cur_L below)
nu     = 2.5;        % alignment strength — editable live (see cur_nu below)
tmax   = 5000;

cell_spd0 = 1;      % default cell speed multiplier

fixframe   = false;
frameinrad = 50;

x_start = -15;      % shared starting x for every target (same vertical line)
vx0     = 0.15;      % default shared horizontal speed of every target

%% ---- Ask user for number of straight-line targets ------------
% Total targets = nRect (straight-line) + 1 (the oscillating one, added
% automatically). nRect may be 0 (just the oscillating target alone).
try
    nRect_answer = inputdlg('Number of straight-line targets (0-5):', 'Setup', 1, {'2'});
catch
    nRect_answer = {};
end
if isempty(nRect_answer)
    nRect = 2;   % default if dialog cancelled or unavailable
else
    nRect = round(str2double(nRect_answer{1}));
    if isnan(nRect), nRect = 2; end
end
nRect  = max(0, min(5, nRect));
nT     = nRect + 1;     % + the oscillating target
oscIdx = nT;            % the oscillating target is always the last one

%% ---- Initial agent conditions -------------------------------
ic_rad = 0.5 * sqrt(N*pi/4/0.91);
rng(2);
X   = ic_rad * (2*rand(N,2) - 1);
rng(18);
ang = 2*pi * rand(N,1);
U   = [cos(ang) sin(ang)];

%% ---- Initial target parameters --------------------------------
% Straight-line targets: evenly spaced parallel horizontal lines.
if nRect == 0
    heights0 = zeros(0,1);
elseif nRect == 1
    heights0 = 0;
else
    heights0 = linspace(-8, 8, nRect)';
end

% Oscillating target: y(t) = H_osc + Amp*sin(phase), phase += Omega each step.
H_osc0    = 0;
Amp0      = 4;
Omega0    = 0.05;

xCommon   = x_start;    % shared x-coordinate of every target (accumulator)
phase_osc = 0;          % oscillation phase accumulator

Tpos = zeros(nT, 2);
for k = 1:nRect
    Tpos(k,:) = [xCommon, heights0(k)];
end
Tpos(oscIdx,:) = [xCommon, H_osc0 + Amp0*sin(phase_osc)];

%% ---- Build initial Target object ----------------------------
tar = Target( mat2cell(Tpos, ones(nT,1), 2) );

%% ---- Preallocate agent variables ----------------------------
U1 = zeros(N,2);

%% ============================================================
%  Build figure + UI
%% ============================================================
colors = lines(nT);

PANEL_H  = min(0.32 + nT*0.05, 0.62);   % panel grows with nT
fig = figure('Name', 'VTP - Aligned Moving Targets', ...
             'NumberTitle', 'off', ...
             'Position', [80 40 980 900]);

ax = axes('Parent', fig, ...
          'Position', [0.05  PANEL_H+0.03  0.90  0.94-PANEL_H]);

pan = uipanel('Parent', fig, ...
              'Title', 'Target Controls', ...
              'Position', [0.01 0.005 0.98 PANEL_H], ...
              'FontSize', 9);

% ---- Cells (agents) speed row ---------------------------------
uicontrol(pan, 'Style','text', ...
    'String', 'Cells', ...
    'FontWeight','bold', 'FontSize', 9, ...
    'Units','normalized', ...
    'Position', [0.01  0.90  0.06  0.08]);

uicontrol(pan,'Style','text','String','Spd:', ...
    'Units','normalized','Position',[0.07 0.90 0.04 0.08],...
    'FontSize',8,'HorizontalAlignment','right');

sld_cell = uicontrol(pan,'Style','slider', ...
    'Min',0,'Max',3,'Value', cell_spd0, ...
    'Units','normalized','Position',[0.12  0.91  0.30  0.06]);

lbl_cell = uicontrol(pan,'Style','text', ...
    'String', sprintf('%.2f', cell_spd0), ...
    'Units','normalized','Position',[0.43 0.90 0.07 0.08],...
    'FontSize',8,'HorizontalAlignment','left');

addlistener(sld_cell,'Value','PostSet', ...
    @(~,~) set(lbl_cell,'String', sprintf('%.2f', sld_cell.Value)));

% ---- Dynamics constants (nu, L) row ----------------------------
% nu = alignment strength, L = interaction length scale — the two
% constants that shape the cell dynamics. Both are just weights/
% scales used every step, so they take effect immediately on
% Apply (no re-seed of agents needed).
uicontrol(pan, 'Style','text', ...
    'String', 'Dynamics', ...
    'FontWeight','bold', 'FontSize', 9, ...
    'Units','normalized', ...
    'Position', [0.01  0.82  0.09  0.08]);

uicontrol(pan,'Style','text','String','nu:', ...
    'Units','normalized','Position',[0.11 0.82 0.05 0.08],...
    'FontSize',8,'HorizontalAlignment','right');

edt_nu = uicontrol(pan,'Style','edit', ...
    'String', num2str(nu), ...
    'Units','normalized','Position',[0.17  0.825  0.13  0.065], ...
    'FontSize',8, 'BackgroundColor', [1 1 1]);

uicontrol(pan,'Style','text','String','L:', ...
    'Units','normalized','Position',[0.35 0.82 0.04 0.08],...
    'FontSize',8,'HorizontalAlignment','right');

edt_L = uicontrol(pan,'Style','edit', ...
    'String', num2str(L), ...
    'Units','normalized','Position',[0.40  0.825  0.13  0.065], ...
    'FontSize',8, 'BackgroundColor', [1 1 1]);

% ---- Motion row: shared horizontal speed (vx) ------------------
% Every target — straight-line or oscillating — uses this SAME speed,
% and they all started on the same vertical line, so they always
% share exactly the same x-coordinate at every step.
uicontrol(pan, 'Style','text', ...
    'String', 'Motion', ...
    'FontWeight','bold', 'FontSize', 9, ...
    'Units','normalized', ...
    'Position', [0.01  0.74  0.09  0.08]);

uicontrol(pan,'Style','text','String','Horiz. speed (vx, shared):', ...
    'Units','normalized','Position',[0.10 0.74 0.24 0.08],...
    'FontSize',8,'HorizontalAlignment','right');

sld_vx = uicontrol(pan,'Style','slider', ...
    'Min',0,'Max',1,'Value', vx0, ...
    'Units','normalized','Position',[0.35  0.75  0.30  0.06]);

lbl_vx = uicontrol(pan,'Style','text', ...
    'String', sprintf('%.3f', vx0), ...
    'Units','normalized','Position',[0.66 0.74 0.08 0.08],...
    'FontSize',8,'HorizontalAlignment','left');

addlistener(sld_vx,'Value','PostSet', ...
    @(~,~) set(lbl_vx,'String', sprintf('%.3f', sld_vx.Value)));

% ---- Per-target rows -------------------------------------------
% Rows 1..nRect: straight-line targets — label + Height slider.
% Row nT (last): the oscillating target — Height (center) + Amp + Omega.
row_h = 0.58 / nT;   % fractional height inside panel, shared by all rows

sld_height = gobjects(nRect,1);
lbl_height = gobjects(nRect,1);
sld_oscH = []; lbl_oscH = [];
sld_oscA = []; lbl_oscA = [];
sld_oscW = []; lbl_oscW = [];

for k = 1:nT
    y0 = 0.74 - k * row_h;   % bottom of this row

    if k <= nRect
        % ---- straight-line target: label + Height ----
        uicontrol(pan, 'Style','text', ...
            'String', sprintf('T%d', k), ...
            'ForegroundColor', colors(k,:), ...
            'FontWeight','bold', 'FontSize', 9, ...
            'Units','normalized', ...
            'Position', [0.01  y0  0.04  row_h*0.7]);

        uicontrol(pan,'Style','text','String','Height:', ...
            'Units','normalized','Position',[0.05 y0 0.08 row_h*0.7],...
            'FontSize',8,'HorizontalAlignment','right');

        sld_height(k) = uicontrol(pan,'Style','slider', ...
            'Min',-15,'Max',15,'Value', heights0(k), ...
            'Units','normalized','Position',[0.13  y0+0.01  0.35  row_h*0.55]);

        lbl_height(k) = uicontrol(pan,'Style','text', ...
            'String', sprintf('%.2f', heights0(k)), ...
            'Units','normalized','Position',[0.49 y0 0.10 row_h*0.7],...
            'FontSize',8,'HorizontalAlignment','left');

        kk = k;   % capture loop variable
        addlistener(sld_height(k),'Value','PostSet', ...
            @(~,~) set(lbl_height(kk),'String', sprintf('%.2f', sld_height(kk).Value)));
    else
        % ---- the oscillating target: Height (center) + Amp + Omega ----
        uicontrol(pan, 'Style','text', ...
            'String', sprintf('T%d~', k), ...
            'ForegroundColor', colors(k,:), ...
            'FontWeight','bold', 'FontSize', 9, ...
            'Units','normalized', ...
            'Position', [0.01  y0  0.045  row_h*0.7]);

        uicontrol(pan,'Style','text','String','H:', ...
            'Units','normalized','Position',[0.06 y0 0.03 row_h*0.7],...
            'FontSize',8,'HorizontalAlignment','right');
        sld_oscH = uicontrol(pan,'Style','slider', ...
            'Min',-15,'Max',15,'Value', H_osc0, ...
            'Units','normalized','Position',[0.09  y0+0.01  0.18  row_h*0.55]);
        lbl_oscH = uicontrol(pan,'Style','text', ...
            'String', sprintf('%.2f', H_osc0), ...
            'Units','normalized','Position',[0.27 y0 0.06 row_h*0.7],...
            'FontSize',8,'HorizontalAlignment','left');

        uicontrol(pan,'Style','text','String','Amp:', ...
            'Units','normalized','Position',[0.34 y0 0.05 row_h*0.7],...
            'FontSize',8,'HorizontalAlignment','right');
        sld_oscA = uicontrol(pan,'Style','slider', ...
            'Min',0,'Max',15,'Value', Amp0, ...
            'Units','normalized','Position',[0.39  y0+0.01  0.18  row_h*0.55]);
        lbl_oscA = uicontrol(pan,'Style','text', ...
            'String', sprintf('%.2f', Amp0), ...
            'Units','normalized','Position',[0.57 y0 0.06 row_h*0.7],...
            'FontSize',8,'HorizontalAlignment','left');

        uicontrol(pan,'Style','text','String','Omega:', ...
            'Units','normalized','Position',[0.64 y0 0.07 row_h*0.7],...
            'FontSize',8,'HorizontalAlignment','right');
        sld_oscW = uicontrol(pan,'Style','slider', ...
            'Min',0,'Max',0.3,'Value', Omega0, ...
            'Units','normalized','Position',[0.71  y0+0.01  0.18  row_h*0.55]);
        lbl_oscW = uicontrol(pan,'Style','text', ...
            'String', sprintf('%.3f', Omega0), ...
            'Units','normalized','Position',[0.89 y0 0.08 row_h*0.7],...
            'FontSize',8,'HorizontalAlignment','left');

        addlistener(sld_oscH,'Value','PostSet', ...
            @(~,~) set(lbl_oscH,'String', sprintf('%.2f', sld_oscH.Value)));
        addlistener(sld_oscA,'Value','PostSet', ...
            @(~,~) set(lbl_oscA,'String', sprintf('%.2f', sld_oscA.Value)));
        addlistener(sld_oscW,'Value','PostSet', ...
            @(~,~) set(lbl_oscW,'String', sprintf('%.3f', sld_oscW.Value)));
    end
end

% ---- Buttons at the bottom of the panel ----------------------
% Callbacks ONLY set a flag via setappdata; no nested functions, no
% closures over script variables needed.
setappdata(fig, 'apply_clicked', false);
setappdata(fig, 'reset_clicked', false);

btn_apply = uicontrol(pan,'Style','pushbutton','String','Apply', ...
    'FontSize',9,'FontWeight','bold', ...
    'Units','normalized','Position',[0.01 0.02 0.12 0.14], ...
    'Callback', @(src,evt) setappdata(fig,'apply_clicked',true));

btn_pause = uicontrol(pan,'Style','togglebutton','String','Pause', ...
    'FontSize',9, ...
    'Units','normalized','Position',[0.15 0.02 0.12 0.14]);

btn_reset = uicontrol(pan,'Style','pushbutton','String','Reset Targets', ...
    'FontSize',9, ...
    'Units','normalized','Position',[0.29 0.02 0.16 0.14], ...
    'Callback', @(src,evt) setappdata(fig,'reset_clicked',true));

%% ---- Mutable simulation variables (plain script variables) --
cur_cell_spd = cell_spd0;
cur_nu       = nu;
cur_L        = L;
cur_vx       = vx0;
cur_height   = heights0;   % nRect x 1
cur_H_osc    = H_osc0;
cur_Amp      = Amp0;
cur_Omega    = Omega0;

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
        cur_cell_spd = get(sld_cell, 'Value');
        cur_vx       = get(sld_vx, 'Value');
        for k = 1:nRect
            cur_height(k) = get(sld_height(k), 'Value');
        end
        if nRect < nT
            cur_H_osc = get(sld_oscH, 'Value');
            cur_Amp   = get(sld_oscA, 'Value');
            cur_Omega = get(sld_oscW, 'Value');
        end

        % ---- Alignment strength nu: applies immediately ----
        newNu = str2double(get(edt_nu, 'String'));
        if isnan(newNu) || newNu < 0
            newNu = cur_nu;   % invalid entry: keep current value
        end
        cur_nu = newNu;
        set(edt_nu, 'String', num2str(cur_nu));

        % ---- Interaction length scale L: applies immediately too ----
        newL = str2double(get(edt_L, 'String'));
        if isnan(newL) || newL <= 0
            newL = cur_L;   % invalid entry: keep current value
        end
        cur_L = newL;
        set(edt_L, 'String', num2str(cur_L));
    end

    % ---- Check Reset button ----
    if getappdata(fig, 'reset_clicked')
        setappdata(fig, 'reset_clicked', false);

        xCommon      = x_start;
        phase_osc    = 0;
        cur_cell_spd = cell_spd0;
        cur_vx       = vx0;
        cur_height   = heights0;
        cur_H_osc    = H_osc0;
        cur_Amp      = Amp0;
        cur_Omega    = Omega0;

        set(sld_cell,'Value', cell_spd0);
        set(lbl_cell,'String', sprintf('%.2f', cell_spd0));
        set(sld_vx,'Value', vx0);
        set(lbl_vx,'String', sprintf('%.3f', vx0));
        for k = 1:nRect
            set(sld_height(k),'Value', heights0(k));
            set(lbl_height(k),'String', sprintf('%.2f', heights0(k)));
        end
        if nRect < nT
            set(sld_oscH,'Value', H_osc0);
            set(lbl_oscH,'String', sprintf('%.2f', H_osc0));
            set(sld_oscA,'Value', Amp0);
            set(lbl_oscA,'String', sprintf('%.2f', Amp0));
            set(sld_oscW,'Value', Omega0);
            set(lbl_oscW,'String', sprintf('%.3f', Omega0));
        end
        t = 0;
    end

    t = t + 1;

    %% -- Move targets (aligned rectilinear + one sinusoidal) --
    % Every target shares the SAME x — one accumulator, not one per
    % target — so they are aligned (same x at every instant) by
    % construction, not just at t=0.
    xCommon   = xCommon + cur_vx;
    phase_osc = phase_osc + cur_Omega;

    for k = 1:nRect
        Tpos(k,:) = [xCommon, cur_height(k)];
    end
    Tpos(oscIdx,:) = [xCommon, cur_H_osc + cur_Amp*sin(phase_osc)];

    tar = Target( mat2cell(Tpos, ones(nT,1), 2) );

    % velocities used only for the on-screen arrows (analytic, not integrated)
    Tvel = zeros(nT,2);
    for k = 1:nRect
        Tvel(k,:) = [cur_vx, 0];
    end
    Tvel(oscIdx,:) = [cur_vx, cur_Amp*cur_Omega*cos(phase_osc)];

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
        % Compute a square frame that contains both agents and targets
        if fixframe
            cx = 0; cy = 0; hw = frameinrad;
        else
            com  = mean(X);
            rmed = sqrt(median((X(:,1)-com(1)).^2 + (X(:,2)-com(2)).^2));
            cx   = com(1);  cy = com(2);  hw = 3*rmed;
        end
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

        % Agents
        scatter(ax, X(:,1), X(:,2), 4, 'k', 'filled');
        quiver(ax, X(:,1), X(:,2), U1(:,1), U1(:,2), ...
               'b', 'AutoScaleFactor', 0.5, 'MaxHeadSize', 0.3);

        % Moving targets
        theta_c = linspace(0, 2*pi, 40);
        r_c     = 0.7;
        vscale  = 8;
        for k = 1:nT
            fill(ax, Tpos(k,1) + r_c*cos(theta_c), ...
                     Tpos(k,2) + r_c*sin(theta_c), ...
                     colors(k,:), 'EdgeColor','none', 'FaceAlpha', 0.9);
            quiver(ax, Tpos(k,1), Tpos(k,2), ...
                       vscale*Tvel(k,1), vscale*Tvel(k,2), ...
                       'off', 'Color', colors(k,:)*0.55, 'LineWidth', 2, ...
                       'MaxHeadSize', 0.6);
            if k == oscIdx
                lbl = sprintf('T%d~', k);
            else
                lbl = sprintf('T%d', k);
            end
            text(ax, Tpos(k,1), Tpos(k,2), lbl, ...
                 'Color','w','FontWeight','bold', ...
                 'HorizontalAlignment','center','FontSize',7);
        end

        set(ax, 'NextPlot', 'replace');
        title(ax, sprintf('t = %d    |    %d aligned targets (%d straight + 1 sine)    |    vx = %.3f, \nu = %.2f, L = %.2f', ...
              t, nT, nRect, cur_vx, cur_nu, cur_L), 'FontSize', 10);
        drawnow limitrate;
    end

    %% -- Update agent state -----------------------------------
    U = cur_cell_spd * tanh(M/cur_L) .* U1;
    X = X + U;

end   % end main loop
