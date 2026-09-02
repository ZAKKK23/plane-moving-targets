%% ============================================================
%  dynamics.m  —  VTP plane simulation with ALIGNED MOVING TARGETS
%  (INVERTED: all oscillating except one straight-line target)
%
%  Same VTP agent dynamics as always (repulsion + alignment + homing
%  over the Delaunay neighbor graph — see neighborhoods.m, alignTo.m,
%  transition.m, voronoiProjectToBoundary.m, Target.m, all unchanged).
%  This is the inverse of the earlier "aligned" version: there, you
%  chose a number of straight-line targets and one oscillating target
%  was always added; here, you choose a number of OSCILLATING targets
%  and one STRAIGHT-LINE target is always added on top. Same variables
%  throughout (vx, nu, L, Height, Amp, Omega), just swapped roles.
%
%    - You choose a number of "oscillating" targets (0-5). Each one
%      moves sinusoidally in y (its own Height/Amplitude/Omega
%      parameters, all independently adjustable) while still moving
%      horizontally at the same constant speed as every other target.
%    - One additional target is always added on top of those: it
%      travels in a perfectly straight horizontal line at a constant
%      height (its own "Height" parameter) — the single straight-line
%      target.
%    - ALL targets — including the straight-line one — share a single
%      horizontal speed (vx) and start on the same vertical line, so
%      they always have exactly the same x-coordinate at every instant.
%      There is only one shared x(t); each target only differs in y(t).
%
%  A control panel lets you:
%    - Change the cells' (agents') overall speed
%    - Change the two constants that govern the cell dynamics: nu
%      (alignment strength) and L (interaction length scale)
%    - Change the shared horizontal speed (vx) of every target
%    - Change each oscillating target's height (oscillation center),
%      amplitude, and omega (angular frequency) — independently, per
%      target
%    - Change the straight-line target's height
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

%% ---- Ask user for number of oscillating targets ---------------
% Total targets = nOsc (oscillating) + 1 (the straight-line one, added
% automatically). nOsc may be 0 (just the straight-line target alone).
try
    nOsc_answer = inputdlg('Number of oscillating targets (0-5):', 'Setup', 1, {'2'});
catch
    nOsc_answer = {};
end
if isempty(nOsc_answer)
    nOsc = 2;   % default if dialog cancelled or unavailable
else
    nOsc = round(str2double(nOsc_answer{1}));
    if isnan(nOsc), nOsc = 2; end
end
nOsc    = max(0, min(5, nOsc));
nT      = nOsc + 1;     % + the straight-line target
rectIdx = nT;           % the straight-line target is always the last one

%% ---- Initial agent conditions -------------------------------
ic_rad = 0.5 * sqrt(N*pi/4/0.91);
rng(2);
X   = ic_rad * (2*rand(N,2) - 1);
rng(18);
ang = 2*pi * rand(N,1);
U   = [cos(ang) sin(ang)];

%% ---- Initial target parameters --------------------------------
% Oscillating targets: y(t) = H_osc(k) + Amp(k)*sin(phase(k)),
% phase(k) += Omega(k) each step — independently for each target.
if nOsc == 0
    H_osc0 = zeros(0,1);
elseif nOsc == 1
    H_osc0 = 0;
else
    H_osc0 = linspace(-8, 8, nOsc)';
end
Amp0   = 4   * ones(nOsc,1);
Omega0 = 0.05 * ones(nOsc,1);

% Straight-line target: a single constant height.
height0_rect = 0;

xCommon   = x_start;        % shared x-coordinate of every target (accumulator)
phase_osc = zeros(nOsc,1);  % one oscillation-phase accumulator per oscillating target

Tpos = zeros(nT, 2);
for k = 1:nOsc
    Tpos(k,:) = [xCommon, H_osc0(k) + Amp0(k)*sin(phase_osc(k))];
end
Tpos(rectIdx,:) = [xCommon, height0_rect];

%% ---- Build initial Target object ----------------------------
tar = Target( mat2cell(Tpos, ones(nT,1), 2) );

%% ---- Preallocate agent variables ----------------------------
U1 = zeros(N,2);

%% ============================================================
%  Build figure + UI
%% ============================================================
colors = lines(nT);

PANEL_H  = min(0.32 + nT*0.05, 0.62);   % panel grows with nT
fig = figure('Name', 'VTP - Aligned Moving Targets (inverted)', ...
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
% Every target — oscillating or straight-line — uses this SAME speed,
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
% Rows 1..nOsc: oscillating targets — label + Height/Amp/Omega, each
% independently adjustable per target.
% Row nT (last): the single straight-line target — label + Height.
row_h = 0.58 / nT;   % fractional height inside panel, shared by all rows

sld_oscH = gobjects(nOsc,1); lbl_oscH = gobjects(nOsc,1);
sld_oscA = gobjects(nOsc,1); lbl_oscA = gobjects(nOsc,1);
sld_oscW = gobjects(nOsc,1); lbl_oscW = gobjects(nOsc,1);
sld_height_rect = []; lbl_height_rect = [];

for k = 1:nT
    y0 = 0.74 - k * row_h;   % bottom of this row

    if k <= nOsc
        % ---- oscillating target k: label + Height/Amp/Omega ----
        uicontrol(pan, 'Style','text', ...
            'String', sprintf('T%d~', k), ...
            'ForegroundColor', colors(k,:), ...
            'FontWeight','bold', 'FontSize', 9, ...
            'Units','normalized', ...
            'Position', [0.01  y0  0.045  row_h*0.7]);

        uicontrol(pan,'Style','text','String','H:', ...
            'Units','normalized','Position',[0.06 y0 0.03 row_h*0.7],...
            'FontSize',8,'HorizontalAlignment','right');
        sld_oscH(k) = uicontrol(pan,'Style','slider', ...
            'Min',-15,'Max',15,'Value', H_osc0(k), ...
            'Units','normalized','Position',[0.09  y0+0.01  0.18  row_h*0.55]);
        lbl_oscH(k) = uicontrol(pan,'Style','text', ...
            'String', sprintf('%.2f', H_osc0(k)), ...
            'Units','normalized','Position',[0.27 y0 0.06 row_h*0.7],...
            'FontSize',8,'HorizontalAlignment','left');

        uicontrol(pan,'Style','text','String','Amp:', ...
            'Units','normalized','Position',[0.34 y0 0.05 row_h*0.7],...
            'FontSize',8,'HorizontalAlignment','right');
        sld_oscA(k) = uicontrol(pan,'Style','slider', ...
            'Min',0,'Max',15,'Value', Amp0(k), ...
            'Units','normalized','Position',[0.39  y0+0.01  0.18  row_h*0.55]);
        lbl_oscA(k) = uicontrol(pan,'Style','text', ...
            'String', sprintf('%.2f', Amp0(k)), ...
            'Units','normalized','Position',[0.57 y0 0.06 row_h*0.7],...
            'FontSize',8,'HorizontalAlignment','left');

        uicontrol(pan,'Style','text','String','Omega:', ...
            'Units','normalized','Position',[0.64 y0 0.07 row_h*0.7],...
            'FontSize',8,'HorizontalAlignment','right');
        sld_oscW(k) = uicontrol(pan,'Style','slider', ...
            'Min',0,'Max',0.3,'Value', Omega0(k), ...
            'Units','normalized','Position',[0.71  y0+0.01  0.18  row_h*0.55]);
        lbl_oscW(k) = uicontrol(pan,'Style','text', ...
            'String', sprintf('%.3f', Omega0(k)), ...
            'Units','normalized','Position',[0.89 y0 0.08 row_h*0.7],...
            'FontSize',8,'HorizontalAlignment','left');

        kk = k;   % capture loop variable
        addlistener(sld_oscH(k),'Value','PostSet', ...
            @(~,~) set(lbl_oscH(kk),'String', sprintf('%.2f', sld_oscH(kk).Value)));
        addlistener(sld_oscA(k),'Value','PostSet', ...
            @(~,~) set(lbl_oscA(kk),'String', sprintf('%.2f', sld_oscA(kk).Value)));
        addlistener(sld_oscW(k),'Value','PostSet', ...
            @(~,~) set(lbl_oscW(kk),'String', sprintf('%.3f', sld_oscW(kk).Value)));
    else
        % ---- the single straight-line target: label + Height ----
        uicontrol(pan, 'Style','text', ...
            'String', sprintf('T%d', k), ...
            'ForegroundColor', colors(k,:), ...
            'FontWeight','bold', 'FontSize', 9, ...
            'Units','normalized', ...
            'Position', [0.01  y0  0.04  row_h*0.7]);

        uicontrol(pan,'Style','text','String','Height:', ...
            'Units','normalized','Position',[0.05 y0 0.08 row_h*0.7],...
            'FontSize',8,'HorizontalAlignment','right');

        sld_height_rect = uicontrol(pan,'Style','slider', ...
            'Min',-15,'Max',15,'Value', height0_rect, ...
            'Units','normalized','Position',[0.13  y0+0.01  0.35  row_h*0.55]);

        lbl_height_rect = uicontrol(pan,'Style','text', ...
            'String', sprintf('%.2f', height0_rect), ...
            'Units','normalized','Position',[0.49 y0 0.10 row_h*0.7],...
            'FontSize',8,'HorizontalAlignment','left');

        addlistener(sld_height_rect,'Value','PostSet', ...
            @(~,~) set(lbl_height_rect,'String', sprintf('%.2f', sld_height_rect.Value)));
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
cur_cell_spd    = cell_spd0;
cur_nu          = nu;
cur_L           = L;
cur_vx          = vx0;
cur_H_osc       = H_osc0;      % nOsc x 1
cur_Amp         = Amp0;        % nOsc x 1
cur_Omega       = Omega0;      % nOsc x 1
cur_height_rect = height0_rect;

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
        for k = 1:nOsc
            cur_H_osc(k) = get(sld_oscH(k), 'Value');
            cur_Amp(k)   = get(sld_oscA(k), 'Value');
            cur_Omega(k) = get(sld_oscW(k), 'Value');
        end
        if nOsc < nT
            cur_height_rect = get(sld_height_rect, 'Value');
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

        xCommon         = x_start;
        phase_osc       = zeros(nOsc,1);
        cur_cell_spd    = cell_spd0;
        cur_vx          = vx0;
        cur_H_osc       = H_osc0;
        cur_Amp         = Amp0;
        cur_Omega       = Omega0;
        cur_height_rect = height0_rect;

        set(sld_cell,'Value', cell_spd0);
        set(lbl_cell,'String', sprintf('%.2f', cell_spd0));
        set(sld_vx,'Value', vx0);
        set(lbl_vx,'String', sprintf('%.3f', vx0));
        for k = 1:nOsc
            set(sld_oscH(k),'Value', H_osc0(k));
            set(lbl_oscH(k),'String', sprintf('%.2f', H_osc0(k)));
            set(sld_oscA(k),'Value', Amp0(k));
            set(lbl_oscA(k),'String', sprintf('%.2f', Amp0(k)));
            set(sld_oscW(k),'Value', Omega0(k));
            set(lbl_oscW(k),'String', sprintf('%.3f', Omega0(k)));
        end
        if nOsc < nT
            set(sld_height_rect,'Value', height0_rect);
            set(lbl_height_rect,'String', sprintf('%.2f', height0_rect));
        end
        t = 0;
    end

    t = t + 1;

    %% -- Move targets (aligned: one straight + rest sinusoidal) --
    % Every target shares the SAME x — one accumulator, not one per
    % target — so they are aligned (same x at every instant) by
    % construction, not just at t=0.
    xCommon = xCommon + cur_vx;
    for k = 1:nOsc
        phase_osc(k) = phase_osc(k) + cur_Omega(k);
        Tpos(k,:)    = [xCommon, cur_H_osc(k) + cur_Amp(k)*sin(phase_osc(k))];
    end
    Tpos(rectIdx,:) = [xCommon, cur_height_rect];

    tar = Target( mat2cell(Tpos, ones(nT,1), 2) );

    % velocities used only for the on-screen arrows (analytic, not integrated)
    Tvel = zeros(nT,2);
    for k = 1:nOsc
        Tvel(k,:) = [cur_vx, cur_Amp(k)*cur_Omega(k)*cos(phase_osc(k))];
    end
    Tvel(rectIdx,:) = [cur_vx, 0];

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
            if k <= nOsc
                lbl = sprintf('T%d~', k);
            else
                lbl = sprintf('T%d', k);
            end
            text(ax, Tpos(k,1), Tpos(k,2), lbl, ...
                 'Color','w','FontWeight','bold', ...
                 'HorizontalAlignment','center','FontSize',7);
        end

        set(ax, 'NextPlot', 'replace');
        title(ax, sprintf('t = %d    |    %d aligned targets (%d sine + 1 straight)    |    vx = %.3f, \nu = %.2f, L = %.2f', ...
              t, nT, nOsc, cur_vx, cur_nu, cur_L), 'FontSize', 10);
        drawnow limitrate;
    end

    %% -- Update agent state -----------------------------------
    U = cur_cell_spd * tanh(M/cur_L) .* U1;
    X = X + U;

end   % end main loop
