%% ============================================================
%  dynamics.m  —  VTP plane simulation with MOVING TARGETS
%
%  Each target undergoes TUSI COUPLE motion: a small circle of radius
%  R/2 rolls without slipping inside a fixed circle of radius R. A
%  point on the rim of the rolling circle then traces a straight line
%  through the center of the fixed circle (the classical Tusi-couple
%  / "hypocycloid" construction). Concretely, target k's position is
%
%       Tpos(k,:) = Tcenter(k,:) + R*cos(phase(k)) * [cos(head(k)) sin(head(k))]
%
%  where Tcenter(k,:) is the fixed pivot (the center of the big
%  circle), head(k) is the orientation of the line of motion, R is the
%  radius of the big circle (== twice the radius of the rolling
%  circle), and phase(k) is the angle of the rolling circle's center,
%  which advances by omega(k) each step. The big/small generating
%  circles are drawn faintly so the mechanism is visible.
%
%  A control panel lets you:
%    - Choose the number of targets (1-6) before starting
%    - Change each target's angular speed (omega) and line orientation
%    - Change the shared oscillation amplitude (big-circle radius)
%    - Change the cells' (agents') overall speed
%    - Change the number of agents N and the two dynamics constants
%      nu (alignment strength) and L (interaction length scale)
%      (applied live, via the Apply button)
%    - Pause / Resume the simulation
%    - Reset targets to new random phases/orientations
%
%  IMPLEMENTATION NOTE: All button callbacks only set a flag via
%  setappdata on the figure; the flags are read back inside the main
%  loop below. This avoids relying on MATLAB's local-function variable
%  scoping in script files, which is not supported consistently across
%  MATLAB versions/configurations.
%% ============================================================

%% ---- Simulation parameters ----------------------------------
N      = 200;
L      = 1;         % interaction length scale — also editable live (see cur_L below)
nu     = 2.5;
tmax   = 5000;

cell_spd0 = 1;      % default cell speed multiplier

fixframe   = false;
frameinrad = 50;

box        = 20;    % soft frame guide (targets stay within tar_amp of their pivot)
tar_rad    = 5;     % initial radial placement of each target's pivot
tar_amp0   = 7;      % default Tusi-couple amplitude (big-circle radius, shared by all targets)

%% ---- Ask user for number of targets -------------------------
try
    nT_answer = inputdlg('Number of targets (1-6):', 'Setup', 1, {'3'});
catch
    nT_answer = {};
end
if isempty(nT_answer)
    nT = 3;   % default if dialog cancelled or unavailable
else
    nT = round(str2double(nT_answer{1}));
    if isnan(nT), nT = 3; end
end
nT = max(1, min(6, nT));

%% ---- Initial agent conditions -------------------------------
ic_rad = 0.5 * sqrt(N*pi/4/0.91);
rng(2);
X   = ic_rad * (2*rand(N,2) - 1);
rng(18);
ang = 2*pi * rand(N,1);
U   = [cos(ang) sin(ang)];

%% ---- Initial target pivots & Tusi-couple state ---------------
rng(7);
tar_ang_init = (2*pi/nT) * (0:nT-1)';
Tcenter      = tar_rad * [cos(tar_ang_init) sin(tar_ang_init)];  % fixed pivots (big-circle centers)
tar_heading  = 2*pi * rand(nT, 1);     % orientation of each target's line of motion
tar_omega0   = 0.05;
tar_omega    = tar_omega0 * ones(nT, 1);   % angular speed of the rolling circle's center
tar_phase    = 2*pi * rand(nT, 1);         % initial phase (angle of rolling-circle center)

% initial target positions from the Tusi-couple formula
Tpos = Tcenter + (tar_amp0 * cos(tar_phase)) .* [cos(tar_heading) sin(tar_heading)];

%% ---- Build initial Target object ----------------------------
tar = Target( mat2cell(Tpos, ones(nT,1), 2) );

%% ---- Preallocate agent variables ----------------------------
U1 = zeros(N,2);

%% ============================================================
%  Build figure + UI
%% ============================================================
colors = lines(nT);

PANEL_H  = min(0.23 + nT*0.07, 0.55);   % panel grows with nT (+ room for cell/swarm rows)
fig = figure('Name', 'VTP - Moving Targets', ...
             'NumberTitle', 'off', ...
             'Position', [80 60 960 820]);

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

% ---- Shared Tusi-couple amplitude row --------------------------
uicontrol(pan, 'Style','text', ...
    'String', 'Amp', ...
    'FontWeight','bold', 'FontSize', 9, ...
    'Units','normalized', ...
    'Position', [0.50  0.90  0.06  0.08]);

sld_amp = uicontrol(pan,'Style','slider', ...
    'Min',0.5,'Max',box,'Value', tar_amp0, ...
    'Units','normalized','Position',[0.58  0.91  0.30  0.06]);

lbl_amp = uicontrol(pan,'Style','text', ...
    'String', sprintf('%.2f', tar_amp0), ...
    'Units','normalized','Position',[0.89 0.90 0.09 0.08],...
    'FontSize',8,'HorizontalAlignment','left');

addlistener(sld_amp,'Value','PostSet', ...
    @(~,~) set(lbl_amp,'String', sprintf('%.2f', sld_amp.Value)));

% ---- Swarm size (N) & the two dynamics constants (nu, L) row ---
% N, nu, and L mirror the live-editable variables in the browser
% demo (docs/plane.html): plain numeric edit boxes, read back and
% validated when Apply is pressed. N re-seeds the agent population;
% nu and L apply immediately (they're just weights/scales used each
% step, no reseed needed).
uicontrol(pan, 'Style','text', ...
    'String', 'Swarm', ...
    'FontWeight','bold', 'FontSize', 9, ...
    'Units','normalized', ...
    'Position', [0.01  0.81  0.08  0.08]);

uicontrol(pan,'Style','text','String','N:', ...
    'Units','normalized','Position',[0.09 0.81 0.04 0.08],...
    'FontSize',8,'HorizontalAlignment','right');

edt_N = uicontrol(pan,'Style','edit', ...
    'String', num2str(N), ...
    'Units','normalized','Position',[0.14  0.815  0.13  0.065], ...
    'FontSize',8, 'BackgroundColor', [1 1 1]);

uicontrol(pan,'Style','text','String','nu:', ...
    'Units','normalized','Position',[0.50 0.81 0.05 0.08],...
    'FontSize',8,'HorizontalAlignment','right');

edt_nu = uicontrol(pan,'Style','edit', ...
    'String', num2str(nu), ...
    'Units','normalized','Position',[0.56  0.815  0.13  0.065], ...
    'FontSize',8, 'BackgroundColor', [1 1 1]);

uicontrol(pan,'Style','text','String','L:', ...
    'Units','normalized','Position',[0.72 0.81 0.04 0.08],...
    'FontSize',8,'HorizontalAlignment','right');

edt_L = uicontrol(pan,'Style','edit', ...
    'String', num2str(L), ...
    'Units','normalized','Position',[0.77  0.815  0.13  0.065], ...
    'FontSize',8, 'BackgroundColor', [1 1 1]);

% ---- Per-target rows -----------------------------------------
% Each row:  coloured label | Speed slider | speed readout | Angle slider | angle readout
row_h   = 0.68 / nT;          % fractional height inside panel
sld_spd = gobjects(nT,1);
lbl_spd = gobjects(nT,1);
sld_ang = gobjects(nT,1);
lbl_ang = gobjects(nT,1);

for k = 1:nT
    y0 = 0.73 - k * row_h;   % bottom of this row (shifted down to leave room for cell/swarm rows)

    % coloured target label
    uicontrol(pan, 'Style','text', ...
        'String', sprintf('T%d', k), ...
        'ForegroundColor', colors(k,:), ...
        'FontWeight','bold', 'FontSize', 9, ...
        'Units','normalized', ...
        'Position', [0.01  y0  0.04  row_h*0.7]);

    % Speed label
    uicontrol(pan,'Style','text','String','omega:', ...
        'Units','normalized','Position',[0.05 y0 0.04 row_h*0.7],...
        'FontSize',8,'HorizontalAlignment','right');

    % Angular-speed (omega) slider
    sld_spd(k) = uicontrol(pan,'Style','slider', ...
        'Min',0,'Max',0.3,'Value', tar_omega(k), ...
        'Units','normalized','Position',[0.10  y0+0.01  0.30  row_h*0.55]);

    % Angular-speed readout
    lbl_spd(k) = uicontrol(pan,'Style','text', ...
        'String', sprintf('%.3f', tar_omega(k)), ...
        'Units','normalized','Position',[0.41 y0 0.07 row_h*0.7],...
        'FontSize',8,'HorizontalAlignment','left');

    % Line-angle label
    uicontrol(pan,'Style','text','String','Line(deg):', ...
        'Units','normalized','Position',[0.49 y0 0.07 row_h*0.7],...
        'FontSize',8,'HorizontalAlignment','right');

    % Angle slider
    sld_ang(k) = uicontrol(pan,'Style','slider', ...
        'Min',0,'Max',360,'Value', rad2deg(tar_heading(k)), ...
        'Units','normalized','Position',[0.57  y0+0.01  0.30  row_h*0.55]);

    % Angle readout
    lbl_ang(k) = uicontrol(pan,'Style','text', ...
        'String', sprintf('%.1f', rad2deg(tar_heading(k))), ...
        'Units','normalized','Position',[0.88 y0 0.07 row_h*0.7],...
        'FontSize',8,'HorizontalAlignment','left');

    % Live readout listeners
    kk = k;   % capture loop variable
    addlistener(sld_spd(k),'Value','PostSet', ...
        @(~,~) set(lbl_spd(kk),'String', sprintf('%.3f', sld_spd(kk).Value)));
    addlistener(sld_ang(k),'Value','PostSet', ...
        @(~,~) set(lbl_ang(kk),'String', sprintf('%.1f', sld_ang(kk).Value)));
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
cur_tar_omega   = tar_omega;
cur_tar_heading = tar_heading;
cur_tar_amp     = tar_amp0;
cur_cell_spd    = cell_spd0;
cur_nu          = nu;
cur_L           = L;

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
        cur_tar_amp  = get(sld_amp, 'Value');
        for k = 1:nT
            cur_tar_omega(k)   = get(sld_spd(k), 'Value');
            cur_tar_heading(k) = deg2rad(get(sld_ang(k), 'Value'));
        end

        % ---- Swarm size N: re-seed the agent population if changed ----
        newN = round(str2double(get(edt_N, 'String')));
        if isnan(newN) || newN <= 0
            newN = N;   % invalid entry: keep current value
        end
        newN = max(3, min(2000, newN));
        if newN ~= N
            N      = newN;
            ic_rad = 0.5 * sqrt(N*pi/4/0.91);
            X      = ic_rad * (2*rand(N,2) - 1);
            ang    = 2*pi * rand(N,1);
            U      = [cos(ang) sin(ang)];
            U1     = zeros(N,2);
        end
        set(edt_N, 'String', num2str(N));   % reflect the validated/clamped value

        % ---- Alignment strength nu: applies immediately, no reseed needed ----
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
        rng('shuffle');
        Tcenter         = tar_rad * [cos(tar_ang_init) sin(tar_ang_init)];
        cur_tar_heading = 2*pi * rand(nT,1);
        cur_tar_omega   = tar_omega0 * ones(nT,1);
        tar_phase       = 2*pi * rand(nT,1);
        cur_tar_amp     = tar_amp0;
        cur_cell_spd    = cell_spd0;
        set(sld_cell,'Value', cell_spd0);
        set(lbl_cell,'String', sprintf('%.2f', cell_spd0));
        set(sld_amp,'Value', tar_amp0);
        set(lbl_amp,'String', sprintf('%.2f', tar_amp0));
        for k = 1:nT
            set(sld_spd(k),'Value', cur_tar_omega(k));
            set(lbl_spd(k),'String', sprintf('%.3f', cur_tar_omega(k)));
            set(sld_ang(k),'Value', rad2deg(cur_tar_heading(k)));
            set(lbl_ang(k),'String', sprintf('%.1f', rad2deg(cur_tar_heading(k))));
        end
        t = 0;
    end

    t = t + 1;

    %% -- Move targets (Tusi couple: rolling circle traces a line) -
    tar_phase = tar_phase + cur_tar_omega;
    dir_vec   = [cos(cur_tar_heading) sin(cur_tar_heading)];      % Nx2 line orientation
    Tpos      = Tcenter + (cur_tar_amp * cos(tar_phase)) .* dir_vec;
    tar_velvec = (-cur_tar_amp * cur_tar_omega .* sin(tar_phase)) .* dir_vec;  % dPos/dt, for the arrows

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

        % Moving targets (Tusi couple)
        theta_c = linspace(0, 2*pi, 40);
        r_c     = 0.7;
        vscale  = 8;
        for k = 1:nT
            R_k     = cur_tar_amp;
            r_k     = R_k / 2;
            dvec    = [cos(cur_tar_heading(k)) sin(cur_tar_heading(k))];

            % fixed big circle (guide) — rotation-invariant, no heading needed
            plot(ax, Tcenter(k,1) + R_k*cos(theta_c), ...
                     Tcenter(k,2) + R_k*sin(theta_c), ...
                     ':', 'Color', colors(k,:), 'LineWidth', 0.75);

            % the diameter line the point travels along
            hline = plot(ax, Tcenter(k,1) + R_k*[-1 1]*dvec(1), ...
                             Tcenter(k,2) + R_k*[-1 1]*dvec(2), ...
                             '-', 'Color', colors(k,:), 'LineWidth', 1);
            hline.Color(4) = 0.35;

            % rolling small circle (radius R/2), center at angle (phase+heading)
            Csmall = Tcenter(k,:) + r_k*[cos(tar_phase(k)+cur_tar_heading(k)) ...
                                          sin(tar_phase(k)+cur_tar_heading(k))];
            plot(ax, Csmall(1) + r_k*cos(theta_c), ...
                     Csmall(2) + r_k*sin(theta_c), ...
                     '-', 'Color', colors(k,:)*0.7, 'LineWidth', 0.75);

            % the target point itself
            fill(ax, Tpos(k,1) + r_c*cos(theta_c), ...
                     Tpos(k,2) + r_c*sin(theta_c), ...
                     colors(k,:), 'EdgeColor','none', 'FaceAlpha', 0.9);
            quiver(ax, Tpos(k,1), Tpos(k,2), ...
                       vscale*tar_velvec(k,1), vscale*tar_velvec(k,2), ...
                       'off', 'Color', colors(k,:)*0.55, 'LineWidth', 2, ...
                       'MaxHeadSize', 0.6);
            text(ax, Tpos(k,1), Tpos(k,2), sprintf('T%d',k), ...
                 'Color','w','FontWeight','bold', ...
                 'HorizontalAlignment','center','FontSize',7);
        end

        set(ax, 'NextPlot', 'replace');
        title(ax, sprintf('t = %d    |    %d moving targets (Tusi couple)    |    N = %d, \\nu = %.2f, L = %.2f', ...
              t, nT, N, cur_nu, cur_L), 'FontSize', 10);
        drawnow limitrate;
    end

    %% -- Update agent state -----------------------------------
    U = cur_cell_spd * tanh(M/cur_L) .* U1;
    X = X + U;

end   % end main loop
