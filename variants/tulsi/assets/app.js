/* ===========================================================
   plane-moving-targets — page glue
   =========================================================== */

/* ---------- tabs ---------- */
const tabButtons = document.querySelectorAll("nav.tabs button");
const views = document.querySelectorAll(".view");
function showView(name) {
  tabButtons.forEach((b) => b.classList.toggle("active", b.dataset.view === name));
  views.forEach((v) => v.classList.toggle("active", v.id === "view-" + name));
  if (name === "sim") sim.canvas.dispatchEvent(new Event("resize-request"));
}
tabButtons.forEach((b) => b.addEventListener("click", () => showView(b.dataset.view)));
window.addEventListener("hashchange", () => {
  const h = location.hash.replace("#", "");
  if (h) showView(h);
});
if (location.hash) showView(location.hash.replace("#", ""));

/* ---------- simulation wiring ---------- */
const canvas = document.getElementById("simCanvas");
const sim = new VTPSim(canvas, 6);

const statsProfileCanvas = document.getElementById("statsProfileCanvas");
const radiusChartsGrid = document.getElementById("radiusChartsGrid");
const btnExportCSV = document.getElementById("btnExportCSV");
const btnExportPNG = document.getElementById("btnExportPNG");
const chartLegend = document.getElementById("chartLegend");

const radiusCanvases = PARAMS.RADII.map((R) => {
  const card = document.createElement("div");
  card.className = "radius-chart-card";
  const cv = document.createElement("canvas");
  card.appendChild(cv);
  radiusChartsGrid.appendChild(card);
  return cv;
});

btnExportCSV.addEventListener("click", () => sim.exportCSV());

// Downloads a single, wide chart: agents within PARAMS.EXPORT_RADIUS (4) of
// each target, over the entire run so far — not just the last on-screen
// window. Drawn at a larger size since it's meant to be read as an image.
// Uses toBlob()+ObjectURL (not toDataURL()+data: URL) because several
// browsers, notably Safari, won't reliably trigger a real download from an
// anchor pointing at a data: URL — it just tries to navigate to it instead.
function exportChartsPNG() {
  const out = document.createElement("canvas");
  out.width = 1400; out.height = 500;
  sim.drawLongRadiusChart(out);

  out.toBlob((blob) => {
    if (!blob) {
      console.error("PNG export failed: canvas.toBlob() returned null");
      return;
    }
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `vtp-radius${PARAMS.EXPORT_RADIUS}-t${sim.t}.png`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  }, "image/png");
}
btnExportPNG.addEventListener("click", exportChartsPNG);

const statusLine = document.getElementById("statusLine");
const targetRows = document.getElementById("targetRows");
const cellSpdSlider = document.getElementById("cellSpd");
const cellSpdVal = document.getElementById("cellSpdVal");
const ampSlider = document.getElementById("amp");
const ampVal = document.getElementById("ampVal");
const nuSlider = document.getElementById("nuSlider");
const nuVal = document.getElementById("nuVal");
const lSlider = document.getElementById("lSlider");
const lVal = document.getElementById("lVal");
const btnPause = document.getElementById("btnPause");
const btnReset = document.getElementById("btnReset");
const numTargetsSelect = document.getElementById("numTargets");

function buildTargetRows(targets) {
  numTargetsSelect.value = String(targets.length);
  targetRows.innerHTML = "";
  chartLegend.innerHTML = targets
    .map((_, k) => `<span><span class="dot" style="background:${TARGET_COLORS[k % TARGET_COLORS.length]}"></span>T${k + 1}</span>`)
    .join("");
  targets.forEach((tg, k) => {
    const row = document.createElement("div");
    row.className = "target-row";
    row.innerHTML = `
      <span class="tname" style="color:${TARGET_COLORS[k % TARGET_COLORS.length]}">T${k + 1}</span>
      <div class="ctrl">omega:
        <input type="range" min="0" max="0.2" step="0.001" value="${tg.omega}" data-k="${k}" data-field="omega" />
        <span class="readout" data-readout="omega-${k}">${tg.omega.toFixed(3)}</span>
      </div>
      <div class="ctrl">Line(deg):
        <input type="range" min="0" max="360" step="0.1" value="${tg.lineDeg}" data-k="${k}" data-field="lineDeg" />
        <span class="readout" data-readout="lineDeg-${k}">${tg.lineDeg.toFixed(1)}</span>
      </div>`;
    targetRows.appendChild(row);
  });
  targetRows.querySelectorAll("input[type=range]").forEach((inp) => {
    inp.addEventListener("input", () => {
      const k = +inp.dataset.k, field = inp.dataset.field;
      const val = +inp.value;
      sim.targets[k][field] = val;
      const ro = targetRows.querySelector(`[data-readout="${field}-${k}"]`);
      ro.textContent = field === "omega" ? val.toFixed(3) : val.toFixed(1);
    });
  });
}

sim.onSetupChange = buildTargetRows;
buildTargetRows(sim.targets);

cellSpdSlider.addEventListener("input", () => {
  sim.cellSpd = +cellSpdSlider.value;
  cellSpdVal.textContent = sim.cellSpd.toFixed(2);
});
ampSlider.addEventListener("input", () => {
  sim.amp = +ampSlider.value;
  ampVal.textContent = sim.amp.toFixed(2);
});
nuSlider.addEventListener("input", () => {
  sim.nu = +nuSlider.value;
  nuVal.textContent = sim.nu.toFixed(2);
});
lSlider.addEventListener("input", () => {
  sim.L = +lSlider.value;
  lVal.textContent = sim.L.toFixed(2);
});

btnPause.addEventListener("click", () => {
  sim.paused = !sim.paused;
  btnPause.textContent = sim.paused ? "Resume" : "Pause";
});
btnReset.addEventListener("click", () => {
  const nT = sim.targets.length;
  sim.setup(nT);
  cellSpdSlider.value = 1; sim.cellSpd = 1; cellSpdVal.textContent = "1.00";
  ampSlider.value = 7; sim.amp = 7; ampVal.textContent = "7.00";
  nuSlider.value = 2.5; sim.nu = 2.5; nuVal.textContent = "2.50";
  lSlider.value = 1; sim.L = 1; lVal.textContent = "1.00";
});
numTargetsSelect.addEventListener("change", () => {
  const n = clamp(Math.round(+numTargetsSelect.value) || 6, 1, 6);
  sim.setup(n);
});

/* ---------- animation loop ---------- */
function loop() {
  sim.step();
  sim.draw();
  sim.drawProfileChart(statsProfileCanvas);
  radiusCanvases.forEach((cv, idx) => sim.drawRadiusChart(cv, idx));
  statusLine.textContent = `t = ${sim.t} | ${sim.targets.length} moving target${sim.targets.length === 1 ? "" : "s"} (Tusi couple) | \u03bd = ${sim.nu.toFixed(2)}, L = ${sim.L.toFixed(2)}`;
  requestAnimationFrame(loop);
}
requestAnimationFrame(loop);

/* ---------- matlab file viewer ---------- */
const FILE_NOTES = {
  "dynamics.m": "Main entry point — sets up the figure/UI and runs the simulation loop.",
  "Target.m": "Target class — target geometry and the homeToTarget homing-vector method.",
  "neighborhoods.m": "Delaunay-graph neighbor lookup.",
  "alignTo.m": "Local alignment force.",
  "transition.m": "Smooth transition/weighting functions (expReciprocal, etc.).",
  "voronoiProjectToBoundary.m": "Projects each agent's movement direction onto its own Voronoi cell boundary.",
  "visualizer.m": "Plotting helper.",
  "planarVoronoiPlot.m": "Plotting helper.",
  "observables.m": "Analysis helper.",
  "buildTable.m": "Batch-run table builder.",
  "serialDynamics.m": "Headless / batch-run version of the simulation loop.",
  "test.m": "Test script.",
  "poly_area.m": "Geometry helper.",
  "poly_area_energy.m": "Geometry helper.",
  "voronoiForwardArea.m": "Geometry / observable helper.",
  "voronoiPressure.m": "Geometry / observable helper.",
  "inwardTotalArea.m": "Geometry / observable helper.",
  "angularMomentum.m": "Observable helper.",
  "ringDists.m": "Observable helper.",
  "boundaryAgents.m": "Geometry helper.",
  "nearestOnSegment.m": "Geometry helper.",
  "rayPolylineIntersect.m": "Geometry helper.",
};
const FILE_ORDER = [
  "dynamics.m", "Target.m", "neighborhoods.m", "alignTo.m", "transition.m",
  "voronoiProjectToBoundary.m", "visualizer.m", "planarVoronoiPlot.m",
  "observables.m", "buildTable.m", "serialDynamics.m", "test.m",
  "poly_area.m", "poly_area_energy.m", "voronoiForwardArea.m", "voronoiPressure.m",
  "inwardTotalArea.m", "angularMomentum.m", "ringDists.m", "boundaryAgents.m",
  "nearestOnSegment.m", "rayPolylineIntersect.m",
];

const fileList = document.getElementById("fileList");
const codeView = document.getElementById("codeView");
const fileHint = document.getElementById("fileHint");
let MATLAB_SRC = null;

async function loadMatlabSource() {
  try {
    const res = await fetch("assets/matlab_src.json");
    MATLAB_SRC = await res.json();
  } catch (e) {
    MATLAB_SRC = null;
  }
  FILE_ORDER.forEach((name, i) => {
    if (!MATLAB_SRC || !(name in MATLAB_SRC)) return;
    const btn = document.createElement("button");
    btn.textContent = name;
    btn.addEventListener("click", () => selectFile(name));
    fileList.appendChild(btn);
    if (i === 0) selectFile(name);
  });
}
function selectFile(name) {
  [...fileList.children].forEach((b) => b.classList.toggle("active", b.textContent === name));
  codeView.textContent = MATLAB_SRC[name];
  fileHint.textContent = FILE_NOTES[name] || "";
}
loadMatlabSource();
