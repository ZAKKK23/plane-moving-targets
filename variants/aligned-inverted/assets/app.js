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
const sim = new VTPSim(canvas, 2);

const statusLine = document.getElementById("statusLine");
const targetRows = document.getElementById("targetRows");
const cellSpdSlider = document.getElementById("cellSpd");
const cellSpdVal = document.getElementById("cellSpdVal");
const nuSlider = document.getElementById("nuSlider");
const nuVal = document.getElementById("nuVal");
const lSlider = document.getElementById("lSlider");
const lVal = document.getElementById("lVal");
const vxSlider = document.getElementById("vxSlider");
const vxVal = document.getElementById("vxVal");
const btnPause = document.getElementById("btnPause");
const btnReset = document.getElementById("btnReset");
const numTargetsSelect = document.getElementById("numTargets");

function buildTargetRows(targets) {
  const nOsc = targets.filter((tg) => tg.osc).length;
  numTargetsSelect.value = String(nOsc);
  targetRows.innerHTML = "";
  targets.forEach((tg, k) => {
    const row = document.createElement("div");
    row.className = "target-row";
    const col = TARGET_COLORS[k % TARGET_COLORS.length];
    if (!tg.osc) {
      row.innerHTML = `
        <span class="tname" style="color:${col}">T${k + 1}</span>
        <div class="ctrl">Height:
          <input type="range" min="-15" max="15" step="0.1" value="${tg.height}" data-k="${k}" data-field="height" />
          <span class="readout" data-readout="height-${k}">${tg.height.toFixed(2)}</span>
        </div>`;
    } else {
      row.innerHTML = `
        <span class="tname" style="color:${col}">T${k + 1}~</span>
        <div class="ctrl">Height:
          <input type="range" min="-15" max="15" step="0.1" value="${tg.height}" data-k="${k}" data-field="height" />
          <span class="readout" data-readout="height-${k}">${tg.height.toFixed(2)}</span>
        </div>
        <div class="ctrl">Amp:
          <input type="range" min="0" max="15" step="0.1" value="${tg.amp}" data-k="${k}" data-field="amp" />
          <span class="readout" data-readout="amp-${k}">${tg.amp.toFixed(2)}</span>
        </div>
        <div class="ctrl">Omega:
          <input type="range" min="0" max="0.3" step="0.001" value="${tg.omega}" data-k="${k}" data-field="omega" />
          <span class="readout" data-readout="omega-${k}">${tg.omega.toFixed(3)}</span>
        </div>`;
    }
    targetRows.appendChild(row);
  });
  targetRows.querySelectorAll("input[type=range]").forEach((inp) => {
    inp.addEventListener("input", () => {
      const k = +inp.dataset.k, field = inp.dataset.field;
      const val = +inp.value;
      sim.targets[k][field] = val;
      const ro = targetRows.querySelector(`[data-readout="${field}-${k}"]`);
      ro.textContent = field === "omega" ? val.toFixed(3) : val.toFixed(2);
    });
  });
}

sim.onSetupChange = buildTargetRows;
buildTargetRows(sim.targets);

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
vxSlider.addEventListener("input", () => {
  sim.vx = +vxSlider.value;
  vxVal.textContent = sim.vx.toFixed(3);
});

btnPause.addEventListener("click", () => {
  sim.paused = !sim.paused;
  btnPause.textContent = sim.paused ? "Resume" : "Pause";
});
btnReset.addEventListener("click", () => {
  const nOsc = sim.nOsc;
  sim.setup(nOsc);
  cellSpdSlider.value = 1; sim.cellSpd = 1; cellSpdVal.textContent = "1.00";
  nuSlider.value = 2.5; sim.nu = 2.5; nuVal.textContent = "2.50";
  lSlider.value = 1; sim.L = 1; lVal.textContent = "1.00";
  vxSlider.value = 0.15; sim.vx = 0.15; vxVal.textContent = "0.150";
});
numTargetsSelect.addEventListener("change", () => {
  const n = clamp(Math.round(+numTargetsSelect.value) || 2, 0, 5);
  sim.setup(n);
});

/* ---------- animation loop ---------- */
function loop() {
  sim.step();
  sim.draw();
  const nOsc = sim.nOsc;
  statusLine.textContent = `t = ${sim.t} | ${sim.targets.length} aligned target${sim.targets.length === 1 ? "" : "s"} (${nOsc} sine + 1 straight) | vx = ${sim.vx.toFixed(3)}, \u03bd = ${sim.nu.toFixed(2)}, L = ${sim.L.toFixed(2)}`;
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
