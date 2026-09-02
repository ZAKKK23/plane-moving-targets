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
const sim = new VTPSim(canvas, 4);

const statusLine = document.getElementById("statusLine");
const cellSpdSlider = document.getElementById("cellSpd");
const cellSpdVal = document.getElementById("cellSpdVal");
const nuSlider = document.getElementById("nuSlider");
const nuVal = document.getElementById("nuVal");
const lSlider = document.getElementById("lSlider");
const lVal = document.getElementById("lVal");
const straightSlider = document.getElementById("straightSlider");
const straightVal = document.getElementById("straightVal");
const radiusSlider = document.getElementById("radiusSlider");
const radiusVal = document.getElementById("radiusVal");
const spacingSlider = document.getElementById("spacingSlider");
const spacingVal = document.getElementById("spacingVal");
const speedSlider = document.getElementById("speedSlider");
const speedVal = document.getElementById("speedVal");
const btnPause = document.getElementById("btnPause");
const btnReset = document.getElementById("btnReset");
const numTargetsSelect = document.getElementById("numTargets");

sim.onSetupChange = (nT) => { numTargetsSelect.value = String(nT); };

cellSpdSlider.addEventListener("input", () => {
  sim.cellSpd = +cellSpdSlider.value;
  cellSpdVal.textContent = sim.cellSpd.toFixed(2);
});
nuSlider.addEventListener("input", () => {
  sim.nu = +nuSlider.value;
  nuVal.textContent = sim.nu.toFixed(2);
});
lSlider.addEventListener("input", () => {
  sim.L = +lSlider.value;
  lVal.textContent = sim.L.toFixed(2);
});
straightSlider.addEventListener("input", () => {
  sim.straightLen = +straightSlider.value;
  straightVal.textContent = sim.straightLen.toFixed(2);
});
radiusSlider.addEventListener("input", () => {
  sim.cornerRadius = +radiusSlider.value;
  radiusVal.textContent = sim.cornerRadius.toFixed(2);
});
spacingSlider.addEventListener("input", () => {
  sim.targetSpacing = +spacingSlider.value;
  spacingVal.textContent = sim.targetSpacing.toFixed(2);
});
speedSlider.addEventListener("input", () => {
  sim.trackSpeed = +speedSlider.value;
  speedVal.textContent = sim.trackSpeed.toFixed(3);
});

btnPause.addEventListener("click", () => {
  sim.paused = !sim.paused;
  btnPause.textContent = sim.paused ? "Resume" : "Pause";
});
btnReset.addEventListener("click", () => {
  sim.setup(sim.nT);
  cellSpdSlider.value = 1; sim.cellSpd = 1; cellSpdVal.textContent = "1.00";
  nuSlider.value = 2.5; sim.nu = 2.5; nuVal.textContent = "2.50";
  lSlider.value = 1; sim.L = 1; lVal.textContent = "1.00";
  straightSlider.value = 10; sim.straightLen = 10; straightVal.textContent = "10.00";
  radiusSlider.value = 5; sim.cornerRadius = 5; radiusVal.textContent = "5.00";
  spacingSlider.value = 8; sim.targetSpacing = 8; spacingVal.textContent = "8.00";
  speedSlider.value = 0.15; sim.trackSpeed = 0.15; speedVal.textContent = "0.150";
});
numTargetsSelect.addEventListener("change", () => {
  const n = clamp(Math.round(+numTargetsSelect.value) || 4, 1, 10);
  sim.setup(n);
});

/* ---------- animation loop ---------- */
function loop() {
  sim.step();
  sim.draw();
  statusLine.textContent = `t = ${sim.t} | ${sim.nT} target${sim.nT === 1 ? "" : "s"} on stadium track | straight = ${sim.straightLen.toFixed(2)}, radius = ${sim.cornerRadius.toFixed(2)}, spacing = ${sim.targetSpacing.toFixed(2)}, speed = ${sim.trackSpeed.toFixed(3)}, \u03bd = ${sim.nu.toFixed(2)}, L = ${sim.L.toFixed(2)}`;
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
