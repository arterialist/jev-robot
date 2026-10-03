const elements = {
  connection: document.querySelector(".connection"),
  connectionLabel: document.querySelector("#connection-label"),
  stream: document.querySelector(".simulation-stage img"),
  activeAction: document.querySelector("#active-action"),
  simClock: document.querySelector("#sim-clock"),
  goalCount: document.querySelector("#goal-count"),
  arenaMap: document.querySelector("#arena-map"),
  goalStatus: document.querySelector("#goal-status"),
  resetRobot: document.querySelector("#reset-robot"),
  robotPosition: document.querySelector("#robot-position"),
  physicsHz: document.querySelector("#physics-hz"),
  physicsTargetHz: document.querySelector("#physics-target-hz"),
  sensoryHz: document.querySelector("#sensory-hz"),
  systemOneHz: document.querySelector("#system-one-hz"),
  systemTwoHz: document.querySelector("#system-two-hz"),
  renderFps: document.querySelector("#render-fps"),
  reflexLag: document.querySelector("#reflex-lag"),
  superseded: document.querySelector("#superseded"),
  targetDistance: document.querySelector("#target-distance"),
  selfThought: document.querySelector("#self-thought"),
  planGoal: document.querySelector("#plan-goal"),
  planStrategy: document.querySelector("#plan-strategy"),
  mindNote: document.querySelector("#mind-note"),
  geminiState: document.querySelector("#gemini-state"),
  jevState: document.querySelector("#jev-state"),
  decisionAction: document.querySelector("#decision-action"),
  decisionConfidence: document.querySelector("#decision-confidence"),
  probabilities: document.querySelector("#probabilities"),
  motorAction: document.querySelector("#motor-action"),
  motorSource: document.querySelector("#motor-source"),
  motorModeNote: document.querySelector("#motor-mode-note"),
  controlModeDescription: document.querySelector("#control-mode-description"),
  motorHz: document.querySelector("#motor-hz"),
  actuatorCount: document.querySelector("#actuator-count"),
  jointTargetCount: document.querySelector("#joint-target-count"),
  jointTargets: document.querySelector("#joint-targets"),
  eventLog: document.querySelector("#event-log"),
  form: document.querySelector("#thought-form"),
  input: document.querySelector("#thought-input"),
  formStatus: document.querySelector("#thought-status"),
  releaseThought: document.querySelector("#release-thought"),
};

let socket;
let reconnectTimer;
let streamRetryTimer;
let latestEventId = -1;
let latestGoalRevision = -1;
let latestArena = null;

function refreshStream() {
  clearTimeout(streamRetryTimer);
  streamRetryTimer = undefined;
  elements.stream.src = `/stream.mjpeg?retry=${Date.now()}`;
}

elements.stream.addEventListener("error", () => {
  if (!streamRetryTimer) streamRetryTimer = setTimeout(refreshStream, 1200);
});

elements.stream.addEventListener("load", () => {
  clearTimeout(streamRetryTimer);
  streamRetryTimer = undefined;
});

function connect() {
  const protocol = window.location.protocol === "https:" ? "wss" : "ws";
  socket = new WebSocket(`${protocol}://${window.location.host}/ws`);

  socket.addEventListener("open", () => {
    elements.connection.className = "connection connected";
    elements.connectionLabel.textContent = "Robot link live";
    elements.input.disabled = false;
    if (!elements.stream.naturalWidth) refreshStream();
  });

  socket.addEventListener("message", (event) => {
    const payload = JSON.parse(event.data);
    if (payload.error) {
      elements.goalStatus.textContent = payload.error;
      return;
    }
    render(payload);
  });

  socket.addEventListener("close", () => {
    elements.connection.className = "connection error";
    elements.connectionLabel.textContent = "Link lost. Retrying";
    elements.input.disabled = true;
    clearTimeout(reconnectTimer);
    reconnectTimer = setTimeout(connect, 1200);
  });

  socket.addEventListener("error", () => socket.close());
}

function setSelfThought(message) {
  const clean = message.trim();
  if (!socket || socket.readyState !== WebSocket.OPEN) {
    elements.formStatus.textContent = "Robot link is not ready. The message was not sent.";
    return;
  }
  socket.send(JSON.stringify({ type: "thought", message: clean }));
  elements.formStatus.textContent = clean
    ? `I am now thinking: ${clean}`
    : "Explicit thought released. Autonomous intent resumed.";
}

function sendControl(payload) {
  if (!socket || socket.readyState !== WebSocket.OPEN) {
    elements.goalStatus.textContent = "Robot link is not ready.";
    return false;
  }
  socket.send(JSON.stringify(payload));
  return true;
}

function render(data) {
  if (!data.sim) return;

  const { sim, decision, motor_action, motor_joint_source, control_mode, goal_revision, camera_view, arena, plan, thought, metrics, models, events } = data;
  const action = decision.action.replaceAll("_", " ").toUpperCase();
  const appliedAction = motor_action.replaceAll("_", " ").toUpperCase();

  elements.activeAction.textContent = appliedAction;
  elements.motorAction.textContent = appliedAction;
  elements.motorHz.textContent = metrics.physics_hz;
  elements.actuatorCount.textContent = metrics.actuator_count;
  elements.motorSource.textContent = control_mode === "primitives" && decision.source === "jev"
    ? "JEV + GAIT"
    : motor_joint_source === "jev" ? "JEV + GAIT" : "LOCAL";
  elements.motorModeNote.textContent = control_mode === "primitives"
    ? "JEV chooses a named movement. Unitree's walking policy handles travel; joint-pose overlays handle gestures, posture, and one small hop."
    : motor_joint_source === "jev"
      ? "JEV sets body-frame walking speed, turning rate, and offsets for all 29 joints. The local gait policy times steps at 50 Hz; PD applies torque at 500 Hz."
      : "JEV is unavailable or pending. The local gait policy moves the robot until a complete JEV command arrives.";
  const jevStatus = !models.jev.configured ? "not configured" : models.jev.error || "pending";
  elements.controlModeDescription.textContent = control_mode === "primitives"
    ? decision.source === "jev"
      ? "JEV chooses a movement; walking and pose policies execute it."
      : `Primitives selected. JEV ${jevStatus}; local gait is active.`
    : motor_joint_source === "jev"
      ? "JEV steers continuous travel and shapes all 29 joints. The local gait policy handles foot timing and balance."
      : `Full control selected. JEV ${jevStatus}; local gait is active.`;
  document.querySelectorAll("[data-control-mode]").forEach((button) => {
    button.setAttribute("aria-pressed", button.dataset.controlMode === control_mode ? "true" : "false");
  });
  elements.jointTargetCount.textContent = `${Object.keys(decision.joint_offsets).length} / ${metrics.actuator_count}`;
  elements.simClock.textContent = `T+${sim.sim_time.toFixed(2).padStart(6, "0")}`;
  elements.goalCount.textContent = `GOAL ${goal_revision}`;
  elements.robotPosition.textContent = `POS ${sim.position.map((value) => value.toFixed(2)).join(" / ")}`;
  elements.physicsHz.textContent = metrics.physics_hz;
  elements.physicsTargetHz.textContent = metrics.physics_timestep_hz;
  elements.sensoryHz.textContent = metrics.sensory_hz;
  elements.systemOneHz.textContent = metrics.system_one_hz.toFixed(1);
  elements.systemTwoHz.textContent = metrics.system_two_hz.toFixed(2);
  elements.renderFps.textContent = metrics.render_fps.toFixed(1);
  elements.reflexLag.textContent = Math.round(metrics.reflex_lag * 1000);
  elements.superseded.textContent = metrics.superseded.toLocaleString();
  elements.targetDistance.textContent = sim.target_distance.toFixed(2);
  latestArena = arena;
  renderMap(arena, sim);
  if (goal_revision !== latestGoalRevision) {
    latestGoalRevision = goal_revision;
    elements.input.value = "";
    elements.goalStatus.textContent = `Goal ${goal_revision} at ${sim.target_position[0].toFixed(2)}, ${sim.target_position[1].toFixed(2)}. Click clear floor to change it.`;
  }
  document.querySelectorAll("[data-camera]").forEach((button) => {
    button.setAttribute("aria-pressed", button.dataset.camera === camera_view ? "true" : "false");
  });
  document.querySelectorAll("[data-thought]").forEach((button) => {
    button.setAttribute("aria-pressed", button.dataset.thought === thought.message ? "true" : "false");
  });

  elements.selfThought.textContent = thought.message || "I am pursuing my autonomous goal.";
  elements.planGoal.textContent = plan.goal;
  elements.planStrategy.textContent = plan.strategy;
  elements.mindNote.textContent = plan.mind_note;
  elements.geminiState.textContent = models.gemini.active
    ? plan.vision_used ? "LIVE + CAMERA" : "LIVE"
    : models.gemini.error
      ? models.gemini.error.toUpperCase()
      : "LOCAL";
  elements.jevState.textContent = decision.source === "jev"
    ? "LIVE"
    : models.jev.error
      ? models.jev.error.toUpperCase()
      : "LOCAL";
  elements.decisionAction.textContent = action;
  elements.decisionConfidence.textContent = `${Math.round(decision.confidence * 100)}% confidence · ${Math.round(decision.latency_ms)} ms`;

  renderProbabilities(decision.probabilities, decision.action, decision.velocity_command, sim.commanded_velocity);
  renderJointTargets(decision.joint_offsets, sim.commanded_joint_targets);
  renderEvents(events);
}

function svgElement(name, attributes) {
  const element = document.createElementNS("http://www.w3.org/2000/svg", name);
  for (const [key, value] of Object.entries(attributes)) element.setAttribute(key, String(value));
  return element;
}

function renderMap(arena, sim) {
  const [lowX, highX, lowY, highY] = arena.bounds;
  const toX = (x) => ((x - lowX) / (highX - lowX)) * 460;
  const toY = (y) => ((highY - y) / (highY - lowY)) * 340;
  const shapes = [];
  for (let x = Math.ceil(lowX); x <= highX; x += 1) {
    shapes.push(svgElement("line", { x1: toX(x), y1: 0, x2: toX(x), y2: 340, stroke: "#28414b", "stroke-width": 1 }));
  }
  for (let y = Math.ceil(lowY); y <= highY; y += 1) {
    shapes.push(svgElement("line", { x1: 0, y1: toY(y), x2: 460, y2: toY(y), stroke: "#28414b", "stroke-width": 1 }));
  }
  shapes.push(svgElement("rect", { x: 2, y: 2, width: 456, height: 336, fill: "none", stroke: "#4b8794", "stroke-width": 4 }));
  for (const obstacle of arena.obstacles) {
    shapes.push(svgElement("rect", {
      x: toX(obstacle.x - obstacle.half_x),
      y: toY(obstacle.y + obstacle.half_y),
      width: (obstacle.half_x * 2 / (highX - lowX)) * 460,
      height: (obstacle.half_y * 2 / (highY - lowY)) * 340,
      fill: "#d65449",
      stroke: "#ff8376",
      "stroke-width": 1,
    }));
  }
  const [goalX, goalY] = sim.target_position;
  shapes.push(svgElement("circle", { cx: toX(goalX), cy: toY(goalY), r: 17, fill: "#72ed8c33", stroke: "#72ed8c", "stroke-width": 2 }));
  shapes.push(svgElement("circle", { cx: toX(goalX), cy: toY(goalY), r: 5, fill: "#72ed8c" }));
  shapes.push(svgElement("polygon", {
    points: "-12,-9 14,0 -12,9",
    transform: `translate(${toX(sim.position[0])} ${toY(sim.position[1])}) rotate(${-sim.orientation_rpy[2] * 180 / Math.PI})`,
    fill: "#eef4ef",
    stroke: "#14252b",
    "stroke-width": 2,
  }));
  elements.arenaMap.replaceChildren(...shapes);
}

function renderJointTargets(offsets, appliedTargets) {
  const entries = Object.entries(offsets);
  elements.jointTargets.replaceChildren();
  if (!entries.length) {
    elements.jointTargets.textContent = "Waiting for Jev's first continuous motor command.";
    return;
  }
  for (const [joint, offset] of entries) {
    const row = document.createElement("div");
    row.className = "joint-target-row";
    const name = document.createElement("span");
    name.textContent = joint.replaceAll("_joint", "").replaceAll("_", " ");
    const value = document.createElement("span");
    const applied = appliedTargets[joint];
    value.textContent = `${offset >= 0 ? "+" : ""}${offset.toFixed(3)} offset · ${Number.isFinite(applied) ? applied.toFixed(2) : "–"} rad applied`;
    row.append(name, value);
    elements.jointTargets.append(row);
  }
}

function renderProbabilities(probabilities, selectedAction, requestedVelocity, appliedVelocity) {
  const entries = Object.entries(probabilities).sort((left, right) => right[1] - left[1]);
  if (entries.length === 0) {
    elements.probabilities.replaceChildren(makeText(
      selectedAction === "direct"
        ? `JEV velocity F ${requestedVelocity[0].toFixed(2)} · L ${requestedVelocity[1].toFixed(2)} m/s · turn ${requestedVelocity[2].toFixed(2)} rad/s. Applied F ${appliedVelocity[0].toFixed(2)} · L ${appliedVelocity[1].toFixed(2)} · turn ${appliedVelocity[2].toFixed(2)}. All 29 joint offsets are below.`
        : "No probability distribution in local fallback mode.",
      "node-value",
    ));
    return;
  }

  const rows = entries.slice(0, 5).map(([name, value]) => {
    const row = document.createElement("div");
    row.className = "probability";
    if (name === selectedAction) row.setAttribute("aria-current", "true");

    const label = makeText(name.replaceAll("_", " "), "probability-name");
    const track = document.createElement("span");
    track.className = "probability-track";
    const fill = document.createElement("span");
    fill.className = "probability-fill";
    fill.style.transform = `scaleX(${Math.max(0, Math.min(1, value))})`;
    track.append(fill);
    const number = makeText(`${Math.round(value * 100)}%`, "probability-value");
    row.append(label, track, number);
    return row;
  });
  elements.probabilities.replaceChildren(...rows);
}

function renderEvents(events) {
  const newest = events.at(-1)?.id ?? -1;
  if (newest === latestEventId) return;
  latestEventId = newest;

  if (events.length === 0) {
    elements.eventLog.replaceChildren(makeText("Waiting for the first event.", "event empty", "li"));
    return;
  }

  const rows = events.slice(-24).map((event) => {
    const row = document.createElement("li");
    row.className = `event ${event.kind}`;
    const timestamp = document.createElement("time");
    timestamp.dateTime = new Date(event.time * 1000).toISOString();
    timestamp.textContent = new Date(event.time * 1000).toLocaleTimeString([], {
      hour: "2-digit",
      minute: "2-digit",
      second: "2-digit",
    });
    row.append(timestamp, makeText(event.message, "event-message"));
    return row;
  });
  elements.eventLog.replaceChildren(...rows);
  elements.eventLog.scrollTop = elements.eventLog.scrollHeight;
}

function makeText(text, className, tag = "span") {
  const element = document.createElement(tag);
  element.className = className;
  element.textContent = text;
  return element;
}

elements.form.addEventListener("submit", (event) => {
  event.preventDefault();
  setSelfThought(elements.input.value);
  elements.input.select();
});

document.querySelectorAll("[data-thought]").forEach((button) => {
  button.addEventListener("click", () => {
    elements.input.value = button.dataset.thought;
    setSelfThought(button.dataset.thought);
  });
});

elements.releaseThought.addEventListener("click", () => {
  elements.input.value = "";
  setSelfThought("");
});

elements.arenaMap.addEventListener("click", (event) => {
  if (!latestArena) return;
  const rect = elements.arenaMap.getBoundingClientRect();
  const [lowX, highX, lowY, highY] = latestArena.bounds;
  const x = lowX + ((event.clientX - rect.left) / rect.width) * (highX - lowX);
  const y = highY - ((event.clientY - rect.top) / rect.height) * (highY - lowY);
  if (sendControl({ type: "goal", x, y })) {
    elements.goalStatus.textContent = `Requesting goal at ${x.toFixed(2)}, ${y.toFixed(2)}...`;
  }
});

document.querySelectorAll("[data-camera]").forEach((button) => {
  button.addEventListener("click", () => sendControl({ type: "camera", view: button.dataset.camera }));
});

document.querySelectorAll("[data-control-mode]").forEach((button) => {
  button.addEventListener("click", () => sendControl({ type: "control_mode", mode: button.dataset.controlMode }));
});

elements.resetRobot.addEventListener("click", () => {
  if (sendControl({ type: "reset" })) elements.goalStatus.textContent = "Resetting robot pose at the arena start...";
});

const keyThoughts = {
  w: "I should go forward carefully",
  a: "I should turn left",
  d: "I should turn right",
  q: "I should step left",
  e: "I should step right",
  s: "I should stop and hold position",
};
document.addEventListener("keydown", (event) => {
  if (event.repeat || event.altKey || event.ctrlKey || event.metaKey) return;
  if (event.target instanceof HTMLInputElement || event.target instanceof HTMLTextAreaElement) return;
  const thought = keyThoughts[event.key.toLowerCase()];
  if (!thought) return;
  event.preventDefault();
  elements.input.value = thought;
  setSelfThought(thought);
});

connect();
