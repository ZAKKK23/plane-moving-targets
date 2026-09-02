/* ===========================================================
   VTP — Aligned Moving Targets  |  live simulation
   Ported from the MATLAB model (dynamics.m / Target.m / alignTo.m /
   neighborhoods.m / voronoiProjectToBoundary.m). Runs the same
   Voronoi-neighbor repulsion + alignment + homing law as the MATLAB
   build. Target motion here: you choose a number of straight-line
   targets (0-5), each on its own fixed-height horizontal line; one
   more target is always added on top that moves sinusoidally in y.
   ALL targets — including the oscillating one — share a single
   horizontal speed (vx) and a single shared x-coordinate, so they are
   aligned (same x at every instant) by construction, not just at t=0.
   =========================================================== */

const TARGET_COLORS = ["#4c8bf5", "#f5a742", "#4cd97d", "#f2685f", "#c792ea", "#3fd0c9"];

const PARAMS = {
  N: 170,             // number of agents ("cells")
  NU_DEFAULT: 2.5,     // default alignment weight (live-editable: sim.nu)
  L_DEFAULT: 1,        // default interaction length scale (live-editable: sim.L)
  VX_DEFAULT: 0.15,    // default shared horizontal speed of every target
  X_START: -15,        // shared starting x for every target (same vertical line)
  HEIGHT_RANGE: 8,     // straight-line targets are spaced across [-HEIGHT_RANGE, HEIGHT_RANGE]
  OSC_AMP_DEFAULT: 4,
  OSC_OMEGA_DEFAULT: 0.05,
};

function expReciprocal(x) {
  if (x >= 1) return 0;
  return Math.exp(-1 / (1 - x)) / (Math.exp(-1 / x) + Math.exp(-1 / (1 - x)));
}
function norm2(v) { return Math.hypot(v[0], v[1]); }
function unit(v) {
  const n = norm2(v);
  return n < 1e-12 ? [0, 0] : [v[0] / n, v[1] / n];
}
function clamp(x, a, b) { return Math.max(a, Math.min(b, x)); }

class VTPSim {
  constructor(canvas, nRect = 2) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.cellSpd = 1.0;
    this.nu = PARAMS.NU_DEFAULT;   // alignment weight — live-editable
    this.L = PARAMS.L_DEFAULT;     // interaction length scale — live-editable
    this.vx = PARAMS.VX_DEFAULT;   // shared horizontal speed — live-editable
    this.paused = false;
    this.t = 0;
    this.cam = null; // {cx,cy,hw} smoothed camera
    this.setup(nRect);
  }

  // nRect = number of straight-line targets (0-5). One more target — the
  // oscillating one — is always added, so total targets = nRect + 1.
  setup(nRect) {
    nRect = clamp(Math.round(nRect), 0, 5);
    this.nRect = nRect;
    const N = PARAMS.N;

    // ---- agents ----
    const icRad = 0.5 * Math.sqrt((N * Math.PI) / 4 / 0.91);
    this.X = new Float64Array(N * 2);
    this.U = new Float64Array(N * 2);
    for (let i = 0; i < N; i++) {
      this.X[2 * i] = icRad * (2 * Math.random() - 1);
      this.X[2 * i + 1] = icRad * (2 * Math.random() - 1);
      const a = 2 * Math.PI * Math.random();
      this.U[2 * i] = Math.cos(a);
      this.U[2 * i + 1] = Math.sin(a);
    }

    // ---- targets: nRect straight horizontal lines + 1 oscillating ----
    this.targets = [];
    const H = PARAMS.HEIGHT_RANGE;
    for (let k = 0; k < nRect; k++) {
      const height = nRect === 1 ? 0 : -H + (2 * H * k) / (nRect - 1);
      this.targets.push({ osc: false, height });
    }
    this.targets.push({
      osc: true,
      height: 0,                          // oscillation center
      amp: PARAMS.OSC_AMP_DEFAULT,
      omega: PARAMS.OSC_OMEGA_DEFAULT,
      phase: 0,
    });
    this.oscIdx = this.targets.length - 1;

    this.xCommon = PARAMS.X_START;   // shared x — one value for every target
    this.t = 0;
    this.cam = null;
    this.onSetupChange && this.onSetupChange(this.targets);
  }

  targetPos(tg) {
    const y = tg.osc ? tg.height + tg.amp * Math.sin(tg.phase) : tg.height;
    return [this.xCommon, y];
  }
  targetVel(tg) {
    const vy = tg.osc ? tg.amp * tg.omega * Math.cos(tg.phase) : 0;
    return [this.vx, vy];
  }

  step() {
    if (this.paused) return;
    this.t++;

    // Every target shares the SAME x — one accumulator, not one per
    // target — so they are aligned (same x at every instant) by
    // construction, not just at t=0.
    this.xCommon += this.vx;
    for (const tg of this.targets) if (tg.osc) tg.phase += tg.omega;

    const N = PARAMS.N;
    const X = this.X, U = this.U;
    const pts = new Array(N);
    for (let i = 0; i < N; i++) pts[i] = [X[2 * i], X[2 * i + 1]];

    const delaunay = d3.Delaunay.from(pts);
    const tpos = this.targets.map((tg) => this.targetPos(tg));

    const newU = new Float64Array(N * 2);
    const nu = this.nu, L = this.L;

    for (let i = 0; i < N; i++) {
      const xi = [X[2 * i], X[2 * i + 1]];
      const neighbors = [...delaunay.neighbors(i)];

      // nearest neighbor + distance
      let dmin = Infinity, nearest = -1;
      for (const j of neighbors) {
        const dx = xi[0] - X[2 * j], dy = xi[1] - X[2 * j + 1];
        const d = Math.hypot(dx, dy);
        if (d < dmin) { dmin = d; nearest = j; }
      }
      if (nearest === -1) { newU[2 * i] = U[2 * i]; newU[2 * i + 1] = U[2 * i + 1]; continue; }

      const s = expReciprocal(dmin / L);

      // repulsion, away from nearest neighbor
      const rrep = unit([xi[0] - X[2 * nearest], xi[1] - X[2 * nearest + 1]]);

      // alignment over Delaunay neighbors
      const ui = unit([U[2 * i], U[2 * i + 1]]);
      let ax = 0, ay = 0;
      for (const j of neighbors) {
        const uj = unit([U[2 * j], U[2 * j + 1]]);
        let dot = ui[0] * uj[0] + ui[1] * uj[1];
        dot = clamp(dot, -1, 1);
        const w = expReciprocal(Math.acos(dot) / Math.PI);
        ax += w * uj[0]; ay += w * uj[1];
      }
      ax /= 6; ay /= 6;

      // homing to nearest target
      let hbest = null, hbestD = Infinity;
      for (const tp of tpos) {
        const v = [tp[0] - xi[0], tp[1] - xi[1]];
        const d2 = v[0] * v[0] + v[1] * v[1];
        if (d2 < hbestD) { hbestD = d2; hbest = v; }
      }
      const hdir = unit(hbest);
      const h = [(1 - s) * hdir[0], (1 - s) * hdir[1]];

      const u1x = (s * rrep[0] + h[0] + nu * ax) / (1 + nu);
      const u1y = (s * rrep[1] + h[1] + nu * ay) / (1 + nu);

      // speed cap: approximates the MATLAB Voronoi ray-to-boundary projection
      // by half the distance to the nearest Delaunay neighbor.
      const M = 0.5 * dmin;
      const speedFactor = this.cellSpd * Math.tanh(M / this.L);

      newU[2 * i] = speedFactor * u1x;
      newU[2 * i + 1] = speedFactor * u1y;
    }

    for (let i = 0; i < N; i++) {
      U[2 * i] = newU[2 * i]; U[2 * i + 1] = newU[2 * i + 1];
      X[2 * i] += U[2 * i]; X[2 * i + 1] += U[2 * i + 1];
    }
  }

  updateCamera() {
    const N = PARAMS.N;
    const X = this.X;
    let sx = 0, sy = 0;
    for (let i = 0; i < N; i++) { sx += X[2 * i]; sy += X[2 * i + 1]; }
    const cx0 = sx / N, cy0 = sy / N;
    let ss = 0;
    for (let i = 0; i < N; i++) {
      const dx = X[2 * i] - cx0, dy = X[2 * i + 1] - cy0;
      ss += dx * dx + dy * dy;
    }
    const rmed = Math.sqrt(ss / N);
    let hw = Math.max(3 * rmed, PARAMS.HEIGHT_RANGE * 1.2);

    let xlo = cx0 - hw, xhi = cx0 + hw, ylo = cy0 - hw, yhi = cy0 + hw;
    for (const tg of this.targets) {
      const [tx, ty] = this.targetPos(tg);
      xlo = Math.min(xlo, tx - 2);
      xhi = Math.max(xhi, tx + 2);
      ylo = Math.min(ylo, ty - 2);
      yhi = Math.max(yhi, ty + 2);
    }
    const hw2 = Math.max(xhi - xlo, yhi - ylo) / 2;
    const cx2 = (xlo + xhi) / 2, cy2 = (ylo + yhi) / 2;

    if (!this.cam) this.cam = { cx: cx2, cy: cy2, hw: hw2 };
    const a = 0.05;
    this.cam.cx += (cx2 - this.cam.cx) * a;
    this.cam.cy += (cy2 - this.cam.cy) * a;
    this.cam.hw += (hw2 - this.cam.hw) * a;
  }

  draw() {
    const canvas = this.canvas, ctx = this.ctx;
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
      canvas.width = w * dpr; canvas.height = h * dpr;
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);

    this.updateCamera();
    const { cx, cy, hw } = this.cam;
    const scale = Math.min(w, h) / (2 * hw);
    const toPx = (x, y) => [w / 2 + (x - cx) * scale, h / 2 - (y - cy) * scale];

    // targets: filled dot, velocity arrow, label ("T3~" for the oscillating one)
    const rC = 0.7 * scale;
    for (let k = 0; k < this.targets.length; k++) {
      const tg = this.targets[k];
      const col = TARGET_COLORS[k % TARGET_COLORS.length];
      const [tgx, tgy] = this.targetPos(tg);
      const [tx, ty] = toPx(tgx, tgy);
      const [vx, vy] = this.targetVel(tg);

      // velocity arrow
      const vscale = 8;
      const [ax1, ay1] = toPx(tgx + vscale * vx, tgy + vscale * vy);
      ctx.strokeStyle = col; ctx.globalAlpha = 0.7; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(tx, ty); ctx.lineTo(ax1, ay1); ctx.stroke();
      ctx.globalAlpha = 1;

      // target dot
      ctx.fillStyle = col;
      ctx.beginPath(); ctx.arc(tx, ty, Math.max(rC, 9), 0, Math.PI * 2); ctx.fill();
      ctx.fillStyle = "#0a0c12";
      ctx.font = "700 9px " + getComputedStyle(document.body).getPropertyValue("--mono");
      ctx.textAlign = "center"; ctx.textBaseline = "middle";
      ctx.fillText((tg.osc ? "T" + (k + 1) + "~" : "T" + (k + 1)), tx, ty + 0.5);
    }

    // agents: dot + velocity arrow
    const N = PARAMS.N;
    ctx.fillStyle = "#f4f5f7";
    for (let i = 0; i < N; i++) {
      const [px, py] = toPx(this.X[2 * i], this.X[2 * i + 1]);
      ctx.beginPath(); ctx.arc(px, py, 2, 0, Math.PI * 2); ctx.fill();
    }
    ctx.strokeStyle = "#5aa4f2"; ctx.fillStyle = "#5aa4f2"; ctx.lineWidth = 1;
    ctx.globalAlpha = 0.85;
    const vscale = 4.2;
    for (let i = 0; i < N; i++) {
      const x0 = this.X[2 * i], y0 = this.X[2 * i + 1];
      const ux = this.U[2 * i], uy = this.U[2 * i + 1];
      const [px0, py0] = toPx(x0, y0);
      const [px1, py1] = toPx(x0 + ux * vscale, y0 + uy * vscale);
      ctx.beginPath(); ctx.moveTo(px0, py0); ctx.lineTo(px1, py1); ctx.stroke();
      const ang = Math.atan2(py1 - py0, px1 - px0);
      ctx.beginPath();
      ctx.moveTo(px1, py1);
      ctx.lineTo(px1 - 4.5 * Math.cos(ang - 0.4), py1 - 4.5 * Math.sin(ang - 0.4));
      ctx.lineTo(px1 - 4.5 * Math.cos(ang + 0.4), py1 - 4.5 * Math.sin(ang + 0.4));
      ctx.closePath(); ctx.fill();
    }
    ctx.globalAlpha = 1;
  }
}
