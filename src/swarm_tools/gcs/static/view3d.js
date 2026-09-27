// 3-D view of the running mission: Cesium terrain, 3-D buildings, the drones as 3-D models at their real heights above
// the ground, the planned route and the obstacles the simulator uses. Reads the same /api/stream as the Mission page.
//
// How the motion stays smooth: the server sends a snapshot every 100 ms; each drone glides from where it is drawn now
// to its newest position over one snapshot interval (so the picture runs about 0.1 s behind the simulation).
// Heights: the simulator's ground is flat, so every drone is drawn at its simulated height above the terrain under it.
(async () => {
  "use strict";
  const $ = (id) => document.getElementById(id);
  const C = Cesium;
  const COLORS = { MASTER: "#4a92ea", FOLLOWER: "#e8eef3", RETIRED: "#e0a33a", DOWN: "#ef6a5f", NORADIO: "#a88ae6" };
  const PHASE = { IDLE: "On ground", TAKEOFF: "Takeoff", CRUISE: "Cruise", HOLD: "At target", LAND: "Landing",
                  LANDED: "Landed", CHARGE: "Charging stop" };
  const MANY = 20;                 // above this many drones: trails and labels only for the leader and for alerts
  let toastTimer = 0;
  const toast = (msg) => { const t = $("toast"); t.textContent = msg; t.hidden = false; clearTimeout(toastTimer);
                          toastTimer = setTimeout(() => { t.hidden = true; }, 2600); };
  const cmd = async (body) => {
    try {
      const r = await fetch("/api/cmd", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
      const j = await r.json(); toast(j.msg || (j.ok ? "Done" : "Failed")); return j;
    } catch (e) { toast("Ground control is not responding"); return { ok: false }; }
  };

  const keys = await (await fetch("/api/config")).json().catch(() => ({}));
  if (!keys.cesium_ion_token) { $("keymsg").hidden = false; return; }
  C.Ion.defaultAccessToken = keys.cesium_ion_token;

  // ------------------------------------------------------------------ viewer, imagery, buildings
  const terrain = C.Terrain.fromWorldTerrain();
  const viewer = new C.Viewer("cesium", {
    terrain, animation: false, timeline: false, geocoder: false, homeButton: false, sceneModePicker: false,
    navigationHelpButton: false, fullscreenButton: false, infoBox: false, selectionIndicator: false,
    baseLayerPicker: false, shadows: false,
  });
  const scene = viewer.scene;
  const drones = new Map();            // id -> drawn state
  window.swarm3d = { viewer, drones };  // handle for debugging in the browser console
  scene.globe.depthTestAgainstTerrain = true;
  scene.postProcessStages.fxaa.enabled = true;
  viewer.targetFrameRate = 40;                                    // enough for smooth motion, spares the laptop GPU
  viewer.clock.currentTime = C.JulianDate.fromIso8601("2026-09-26T10:30:00Z");   // fixed sun: mid-afternoon in Pakistan
  viewer.clock.shouldAnimate = false;
  if (keys.mapbox_token) {   // sharper imagery with the user's Mapbox token (Cesium's default imagery otherwise)
    viewer.imageryLayers.removeAll();
    viewer.imageryLayers.addImageryProvider(new C.UrlTemplateImageryProvider({
      url: `https://api.mapbox.com/v4/mapbox.satellite/{z}/{x}/{y}@2x.jpg90?access_token=${encodeURIComponent(keys.mapbox_token)}`,
      maximumLevel: 20, credit: new C.Credit("© Mapbox © OpenStreetMap © Maxar", true) }));
  }
  let osmBuildings = null, photo = null;
  try { osmBuildings = await C.createOsmBuildingsAsync(); scene.primitives.add(osmBuildings); }
  catch (e) { toast("3-D buildings could not load"); }
  $("buildings").addEventListener("change", (ev) => { if (osmBuildings) osmBuildings.show = ev.target.checked && !$("photo").checked; });
  $("photo").addEventListener("change", async (ev) => {
    if (ev.target.checked && !photo) {
      try { photo = await C.createGooglePhotorealistic3DTileset(); scene.primitives.add(photo); }
      catch (e) { toast("Photorealistic tiles are not available for this token"); ev.target.checked = false; return; }
    }
    if (photo) photo.show = ev.target.checked;
    scene.globe.show = !ev.target.checked;           // the photorealistic tiles contain their own ground
    if (osmBuildings) osmBuildings.show = !ev.target.checked && $("buildings").checked;
  });
  $("shadows").addEventListener("change", (ev) => { viewer.shadows = ev.target.checked; });

  // Ground height under the drones. The globe answers instantly for loaded terrain tiles; while the photorealistic
  // tiles hide the globe (or a tile is not loaded yet) a slower terrain query under the leader fills in.
  let baseGround = null, groundBusy = false, groundAt = 0;
  terrain.readyEvent.addEventListener(() => { groundAt = 0; });
  function refreshBaseGround(lat, lon) {
    if (groundBusy || performance.now() - groundAt < 2000 || !terrain.ready) return;
    groundBusy = true; groundAt = performance.now();
    C.sampleTerrainMostDetailed(viewer.terrainProvider, [C.Cartographic.fromDegrees(lon, lat)])
      .then(([c]) => { if (Number.isFinite(c.height)) baseGround = c.height; })
      .catch(() => {}).finally(() => { groundBusy = false; });
  }
  const scratchCarto = new C.Cartographic();
  function groundHeight(lat, lon, previous) {
    let g;
    if (scene.globe.show) g = scene.globe.getHeight(C.Cartographic.fromDegrees(lon, lat, 0, scratchCarto));
    if (g === undefined) g = previous ?? baseGround;
    if (g == null) return null;               // not known yet: a drone is drawn once the terrain under it has loaded
    return previous == null ? g : previous + (g - previous) * 0.35;    // smooth the steps when finer tiles arrive
  }

  // ------------------------------------------------------------------ route, home/target pins, stops, obstacles
  const routeEnt = viewer.entities.add({ polyline: { positions: [], width: 5, clampToGround: true,
    material: C.Color.fromCssColorString("#ffd166").withAlpha(0.85) } });
  const pins = {};
  const pin = (name, text, color) => viewer.entities.add({ name, position: C.Cartesian3.fromDegrees(0, 0),
    point: { pixelSize: 11, color: C.Color.fromCssColorString(color), outlineColor: C.Color.WHITE, outlineWidth: 2,
             heightReference: C.HeightReference.CLAMP_TO_GROUND, disableDepthTestDistance: Number.POSITIVE_INFINITY },
    label: { text, font: "600 15px Barlow Condensed, sans-serif", fillColor: C.Color.WHITE, showBackground: true,
             backgroundColor: C.Color.fromCssColorString(color).withAlpha(0.9), pixelOffset: new C.Cartesian2(0, -24),
             heightReference: C.HeightReference.CLAMP_TO_GROUND, disableDepthTestDistance: Number.POSITIVE_INFINITY } });
  pins.home = pin("home", "HOME", "#2d3b46");
  pins.target = pin("target", "TARGET", "#c0392b");

  const obstacleSource = new C.CustomDataSource("obstacles");
  viewer.dataSources.add(obstacleSource);
  $("obst").addEventListener("change", (ev) => { obstacleSource.show = ev.target.checked; });
  let obstaclesVersion = -1, obstacleAll = [], drawnAt = null;
  const NEAR_DEG = 0.02;          // long routes: draw only obstacles within ~2 km of the leader, refreshed as it moves
  function drawObstacles(center) {
    obstacleSource.entities.removeAll();
    const near = obstacleAll.length <= 3000 || !center ? obstacleAll :
      obstacleAll.filter((o) => Math.abs(o.lat - center[0]) < NEAR_DEG && Math.abs(o.lon - center[1]) < NEAR_DEG);
    for (const o of near) {
      obstacleSource.entities.add({ polygon: {
        hierarchy: C.Cartesian3.fromDegreesArray(o.poly.flatMap(([lat, lon]) => [lon, lat])),
        material: C.Color.fromCssColorString(o.kind === "wood" ? "#3fbf86" : "#ef6a5f").withAlpha(0.42),
        classificationType: C.ClassificationType.BOTH } });
    }
    drawnAt = center;
  }
  async function loadObstacles(v, center) {
    obstaclesVersion = v;
    const o = await (await fetch("/api/obstacles")).json();
    obstacleAll = o.polygons.map((poly, k) => ({ poly, kind: o.kinds[k], lat: poly[0][0], lon: poly[0][1] }));
    drawObstacles(center);
  }

  const stopSource = new C.CustomDataSource("stops");
  viewer.dataSources.add(stopSource);
  let stopsKey = "";
  function drawStops(s) {
    const key = `${(s.stops || []).length}:${s.next_stop}`;
    if (key === stopsKey) return;
    stopsKey = key;
    stopSource.entities.removeAll();
    (s.stops || []).forEach(([lat, lon], k) => stopSource.entities.add({ position: C.Cartesian3.fromDegrees(lon, lat),
      point: { pixelSize: 9, color: C.Color.fromCssColorString(s.next_stop != null && k < s.next_stop ? "#6b7b87" : "#e0a33a"),
               outlineColor: C.Color.WHITE, outlineWidth: 2, heightReference: C.HeightReference.CLAMP_TO_GROUND,
               disableDepthTestDistance: Number.POSITIVE_INFINITY },
      label: { text: `Stop ${k + 1}`, font: "500 12px Barlow, sans-serif", pixelOffset: new C.Cartesian2(0, -16),
               showBackground: true, heightReference: C.HeightReference.CLAMP_TO_GROUND,
               distanceDisplayCondition: new C.DistanceDisplayCondition(0, 5000),
               disableDepthTestDistance: Number.POSITIVE_INFINITY } }));
  }

  // ------------------------------------------------------------------ drones
  const svg = (body, size = 96) => "data:image/svg+xml;charset=utf-8," + encodeURIComponent(
    `<svg xmlns="http://www.w3.org/2000/svg" viewBox="-50 -50 100 100" width="${size}" height="${size}">${body}</svg>`);
  const HALO = svg(`<circle r="40" fill="none" stroke="#4a92ea" stroke-width="5" stroke-opacity="0.95"/>
    <circle r="46" fill="none" stroke="#4a92ea" stroke-width="3" stroke-opacity="0.35"/>`);
  const CRASH = svg(`<circle r="30" fill="#ef6a5f" fill-opacity="0.25" stroke="#ef6a5f" stroke-width="4"/>
    <path d="M-14 -14L14 14M14 -14L-14 14" stroke="#ef6a5f" stroke-width="8" stroke-linecap="round"/>`, 64);
  let span = 100;                      // measured time between snapshots (ms)
  let lastMsgAt = 0;
  const lerpAngle = (a, b, f) => a + ((((b - a) % 360) + 540) % 360 - 180) * f;
  const frac = (e, now) => Math.min(1, Math.max(0, (now - e.t0) / span));
  function posNow(e, now, result) { return C.Cartesian3.lerp(e.from, e.to, frac(e, now), result || new C.Cartesian3()); }
  function roleOf(d) {
    if (d.role === "DOWN") return "DOWN";
    if (!d.radio_ok) return "NORADIO";
    return d.role in COLORS ? d.role : "FOLLOWER";
  }
  function styleDrone(e) {
    const c = C.Color.fromCssColorString(COLORS[e.role]);
    const m = e.ent.model;
    m.color = c;                                   // multiplies the white body; the dark arms stay dark
    m.colorBlendMode = C.ColorBlendMode.HIGHLIGHT;
    m.silhouetteColor = e.role === "FOLLOWER" ? C.Color.fromCssColorString("#0b1116").withAlpha(0.8) : c;
    m.silhouetteSize = e.role === "FOLLOWER" ? 1 : 2;
    e.trail.polyline.width = e.role === "FOLLOWER" ? 2 : 5;
    e.trail.polyline.material = e.role === "FOLLOWER" ? c.withAlpha(0.35)
      : new C.PolylineGlowMaterialProperty({ color: c.withAlpha(0.9), glowPower: 0.2 });
    e.stem.polyline.material = c.withAlpha(0.25);
    e.crash.show = e.role === "DOWN";
  }
  function makeDrone(d, pos) {
    const e = { id: d.id, from: pos, to: pos, t0: performance.now(), h0: d.hdg, h1: d.hdg, ground: null, d,
                role: null, trail: null, points: [], lastPoint: null, trailPos: [], trailGround: null, trailDirty: false };
    const hpr = new C.HeadingPitchRoll();
    const Callback = C.CallbackPositionProperty || C.CallbackProperty;
    e.position = new Callback((time, result) => posNow(e, performance.now(), result), false);
    e.ent = viewer.entities.add({
      position: e.position,
      orientation: new C.CallbackProperty((time, result) => {
        const now = performance.now(), f = frac(e, now), dd = e.d;
        const hdg = lerpAngle(e.h0, e.h1, f);
        hpr.heading = C.Math.toRadians(hdg - 90);                    // model nose is +x; Cesium heading 0 = east
        hpr.pitch = dd.role === "DOWN" ? 0 : -Math.min(dd.speed * 0.022, 0.26);   // nose down when flying forward
        hpr.roll = dd.role === "DOWN" ? (dd.status === "falling" ? (now / 180) % (2 * Math.PI) : 1.2) : 0;
        return C.Transforms.headingPitchRollQuaternion(posNow(e, now, e.scratch), hpr, C.Ellipsoid.WGS84, undefined, result);
      }, false),
      model: { uri: "/static/drone.glb", minimumPixelSize: 60, maximumScale: 400, scale: 1.4,
               shadows: C.ShadowMode.ENABLED },
      label: { text: "", font: "600 13px IBM Plex Mono, monospace", fillColor: C.Color.WHITE, showBackground: true,
               backgroundColor: C.Color.fromCssColorString("#0a1015").withAlpha(0.78), pixelOffset: new C.Cartesian2(20, -20),
               disableDepthTestDistance: Number.POSITIVE_INFINITY, horizontalOrigin: C.HorizontalOrigin.LEFT },
    });
    e.scratch = new C.Cartesian3();
    e.halo = viewer.entities.add({ position: e.position, show: false,
      billboard: { image: HALO, width: 96, height: 96, disableDepthTestDistance: Number.POSITIVE_INFINITY,
                   scale: new C.CallbackProperty(() => 1 + 0.12 * Math.sin(performance.now() / 230), false) } });
    e.crash = viewer.entities.add({ position: e.position, show: false,
      billboard: { image: CRASH, width: 34, height: 34, disableDepthTestDistance: Number.POSITIVE_INFINITY } });
    // the trail is stored as height above the ground, so a finer terrain tile arriving later cannot bend it
    e.trail = viewer.entities.add({ polyline: { width: 3, positions: new C.CallbackProperty(() => {
      if (e.points.length < 2) return [];
      if (e.trailDirty || Math.abs(e.trailGround - e.ground) > 0.5) {
        e.trailGround = e.ground; e.trailDirty = false;
        e.trailPos = e.points.map(([lon, lat, alt]) => C.Cartesian3.fromDegrees(lon, lat, e.ground + alt));
      }
      return e.trailPos.concat([posNow(e, performance.now())]);
    }, false) } });
    const groundPt = new C.Cartographic();
    e.stem = viewer.entities.add({ polyline: { width: 1.5, positions: new C.CallbackProperty(() => {
      const p = posNow(e, performance.now());
      C.Cartographic.fromCartesian(p, C.Ellipsoid.WGS84, groundPt);
      groundPt.height = e.ground ?? groundPt.height;
      return [C.Cartographic.toCartesian(groundPt), p];
    }, false) } });
    return e;
  }
  function removeAllDrones() {
    for (const e of drones.values()) for (const k of ["ent", "halo", "crash", "trail", "stem"]) viewer.entities.remove(e[k]);
    drones.clear();
  }

  // V formation lines: leader -> nearest follower behind it on each side -> the next one, and so on.
  const arms = { left: [], right: [] };
  const vMaterial = C.Color.fromCssColorString("#7fd8ff").withAlpha(0.6);
  const armLine = (side) => viewer.entities.add({ polyline: { width: 2.2, material: vMaterial,
    positions: new C.CallbackProperty(() => {
      const now = performance.now();
      return arms[side].length > 1 ? arms[side].map((e) => posNow(e, now)) : [];
    }, false) } });
  const vLines = [armLine("left"), armLine("right")];
  function computeArms(s, lead) {
    arms.left = []; arms.right = [];
    const le = lead && drones.get(lead.id);
    if (!le || lead.landed) return;
    const h = C.Math.toRadians(lead.hdg), fx = Math.sin(h), fy = Math.cos(h);     // forward, east/north
    const kx = Math.cos(C.Math.toRadians(lead.lat)) * 111320, ky = 110540;
    const left = [], right = [];
    for (const d of s.drones) {
      if (d.id === lead.id || d.role !== "FOLLOWER" || d.landed || d.master !== lead.id || !drones.has(d.id)) continue;
      const x = (d.lon - lead.lon) * kx, y = (d.lat - lead.lat) * ky;
      const along = x * fx + y * fy, cross = -x * fy + y * fx;            // cross > 0: left of the leader
      if (along > 5 || Math.hypot(x, y) > 400) continue;                   // not in the V (overtaking, far away)
      (cross >= 0 ? left : right).push({ e: drones.get(d.id), along });
    }
    const chain = (arr) => [le, ...arr.sort((a, b) => b.along - a.along).map((a) => a.e)];
    arms.left = chain(left); arms.right = chain(right);
  }

  // ------------------------------------------------------------------ events: banner and log
  let lastSeq = null, bannerTimer = 0;
  function banner(kind, text) {
    const b = $("banner");
    b.className = `banner ${kind}`; b.textContent = text; b.hidden = false;
    clearTimeout(bannerTimer);
    bannerTimer = setTimeout(() => { b.hidden = true; }, 5200);
  }
  function showEvents(s) {
    const evs = s.events || [];
    const newest = evs.length ? evs[evs.length - 1][0] : 0;
    if (lastSeq !== null && newest > lastSeq) {
      const fresh = evs.filter((ev) => ev[0] > lastSeq);
      const big = fresh.filter((ev) => ev[2] === "master" || ev[2] === "fault");
      if (big.length) { const ev = big[big.length - 1]; banner(ev[2], ev[3]); }
    }
    if (newest !== lastSeq) {
      lastSeq = newest;
      $("log").innerHTML = evs.slice(-6).reverse().map(([, t, kind, text]) =>
        `<li class="${kind}"><span class="t">${t.toFixed(0)} s</span><span class="m">${text.replace(/[<&]/g, (c) => ({ "<": "&lt;", "&": "&amp;" })[c])}</span></li>`).join("");
    }
  }

  // ------------------------------------------------------------------ snapshot -> scene
  let lastRouteKey = "", state = null, lastT = -1, missionKey = "";
  const fmt = (v, unit, digits = 0) => (v == null ? "—" : `${Number(v).toFixed(digits)} ${unit}`);
  function update(s) {
    state = s;
    const now = performance.now();
    if (lastMsgAt) span = Math.min(400, Math.max(40, span * 0.8 + (now - lastMsgAt) * 0.2));
    lastMsgAt = now;
    const mk = `${s.params.home}|${s.params.target}|${s.total}|${s.params.seed}`;
    if (mk !== missionKey || s.t < lastT - 0.5) { removeAllDrones(); missionKey = mk; stopsKey = ""; }
    lastT = s.t;
    $("clock").textContent = `t = ${s.t.toFixed(0)} s`;
    $("r_phase").textContent = s.phase ? (PHASE[s.phase] || s.phase) : "No leader";
    $("r_master").textContent = s.master ? `Drone ${s.master}` : "electing…";
    $("r_alive").textContent = `${s.alive} / ${s.total}`;
    $("r_hits").textContent = s.route ? String(s.hits) : "—";
    $("r_left").textContent = s.dist_left == null ? "—" : s.dist_left >= 2000 ? `${(s.dist_left / 1000).toFixed(1)} km` : `${s.dist_left} m`;
    $("r_rms").textContent = fmt(s.rms, "m", 1);
    $("r_sep").textContent = fmt(s.min_sep, "m", 1);
    $("r_sep").classList.toggle("bad", s.min_sep != null && s.min_sep < 5);
    $("pause").textContent = s.running ? "Pause" : "Resume";
    document.querySelectorAll(".speeds button").forEach((b) => b.setAttribute("aria-pressed", String(Number(b.dataset.speed) === s.speed)));
    const home = s.params.home, tgt = s.params.target;
    pins.home.position = C.Cartesian3.fromDegrees(home[1], home[0]);
    pins.target.position = C.Cartesian3.fromDegrees(tgt[1], tgt[0]);
    const rk = s.route ? `${s.route.length}:${s.route[0]}` : `${home}:${tgt}`;
    if (rk !== lastRouteKey) {
      lastRouteKey = rk;
      const pts = s.route || [home, tgt];
      routeEnt.polyline.positions = C.Cartesian3.fromDegreesArray(pts.flatMap(([lat, lon]) => [lon, lat]));
    }
    const lead = s.drones.find((d) => d.id === s.master) || null;
    const ref = lead || s.drones.find((d) => d.role !== "DOWN") || s.drones[0];
    const center = ref ? [ref.lat, ref.lon] : null;
    if (center) refreshBaseGround(center[0], center[1]);
    if (s.obstacles_version !== obstaclesVersion) loadObstacles(s.obstacles_version, center).catch(() => {});
    else if (obstacleAll.length > 3000 && center && (!drawnAt || Math.abs(center[0] - drawnAt[0]) + Math.abs(center[1] - drawnAt[1]) > NEAR_DEG / 2)) {
      drawObstacles(center);
    }
    drawStops(s);
    const many = s.drones.length > MANY, trailsOn = $("trails").checked, labelsOn = $("labels").checked;
    for (const d of s.drones) {
      let e = drones.get(d.id);
      const ground = groundHeight(d.lat, d.lon, e ? e.ground : null);
      if (ground == null) continue;
      const pos = C.Cartesian3.fromDegrees(d.lon, d.lat, ground + Math.max(d.alt, 0));
      if (!e) { e = makeDrone(d, pos); drones.set(d.id, e); }
      e.ground = ground;
      e.from = posNow(e, now); e.to = pos; e.t0 = now;              // glide from the drawn position to the new one
      e.h0 = lerpAngle(e.h0, e.h1, 1); e.h1 = d.hdg;
      e.d = d;
      const role = roleOf(d);
      if (role !== e.role) { e.role = role; styleDrone(e); }
      const isLead = d.id === s.master;
      e.halo.show = isLead && !d.landed;
      // trail: a point every 2 m; the leader keeps the last 150 m, the others a short 24 m streak; not on the ground
      if (!d.landed && (!e.lastPoint || C.Cartesian3.distance(e.lastPoint, pos) > 2)) {
        e.points.push([d.lon, d.lat, Math.max(d.alt, 0)]); e.lastPoint = pos; e.trailDirty = true;
      }
      while (e.points.length > (isLead ? 75 : 12)) { e.points.shift(); e.trailDirty = true; }
      e.trail.show = trailsOn && (!many || isLead) && e.points.length > 1;
      e.stem.show = !d.landed && d.alt > 1 && (!many || isLead);
      const alert = role !== "FOLLOWER" && role !== "MASTER";
      const lb = e.ent.label;
      lb.show = labelsOn && (isLead || alert || !many);
      const what = role === "DOWN" ? "DOWN" : role === "NORADIO" ? "no radio" : role === "RETIRED" ? "leaving" :
        d.landed ? (s.phase === "CHARGE" ? `charging ${d.battery.toFixed(0)} %` : "landed") : `${d.alt.toFixed(0)} m`;
      lb.text = isLead ? `★ ${d.id} LEADER · ${what} · ${d.battery.toFixed(0)} %` : alert || d.landed ? `${d.id} · ${what}` : `${d.id}`;
      lb.fillColor = C.Color.fromCssColorString(isLead ? "#9cc4f5" : alert ? COLORS[role] : "#ffffff");
      lb.distanceDisplayCondition = isLead || alert ? undefined : new C.DistanceDisplayCondition(0, 260);
    }
    for (const [id, e] of drones) if (!s.drones.some((d) => d.id === id)) {
      for (const k of ["ent", "halo", "crash", "trail", "stem"]) viewer.entities.remove(e[k]);
      drones.delete(id);
    }
    computeArms(s, lead);
    vLines.forEach((l) => { l.show = $("vlines").checked; });
    $("r_batt").textContent = lead ? `${lead.battery.toFixed(0)} %` : "—";
    showEvents(s);
    if (!camInit && ref) { camInit = true; setMode(camMode); }
  }

  // ------------------------------------------------------------------ camera: chase, orbit, top, free
  // In the follow modes the camera is placed every frame around the swarm centre; drag to turn it, wheel to zoom.
  let camMode = "chase", camInit = false, camHeading = null, camRange = 4000, userYaw = 0, userPitch = 0, zoomMul = 1;
  let lift = 0;                        // extra downward look (degrees) that keeps the camera above hills and rooftops
  const ssc = scene.screenSpaceCameraController;
  function setMode(mode) {
    camMode = mode;
    document.querySelectorAll(".cams button").forEach((b) => b.setAttribute("aria-pressed", String(b.dataset.cam === mode)));
    userYaw = 0; userPitch = 0; zoomMul = 1;
    if (mode === "free") {
      viewer.camera.lookAtTransform(C.Matrix4.IDENTITY);
      ssc.enableInputs = true;
    } else {
      ssc.enableInputs = false;
    }
  }
  document.querySelectorAll(".cams button").forEach((b) => b.addEventListener("click", () => setMode(b.dataset.cam)));
  const canvas = viewer.canvas;
  let drag = null;
  canvas.addEventListener("pointerdown", (ev) => { if (camMode !== "free") drag = { x: ev.clientX, y: ev.clientY }; });
  window.addEventListener("pointerup", () => { drag = null; });
  window.addEventListener("pointermove", (ev) => {
    if (!drag || camMode === "free") return;
    userYaw += (ev.clientX - drag.x) * 0.3;
    userPitch = Math.max(-60, Math.min(35, userPitch + (ev.clientY - drag.y) * 0.25));
    drag = { x: ev.clientX, y: ev.clientY };
  });
  canvas.addEventListener("wheel", (ev) => {
    if (camMode === "free") return;
    ev.preventDefault();
    zoomMul = Math.max(0.15, Math.min(12, zoomMul * Math.exp(ev.deltaY * 0.0012)));
  }, { passive: false });

  const centre = new C.Cartesian3();
  let lastFrame = performance.now();
  scene.preRender.addEventListener(() => {
    const now = performance.now(), dt = Math.min(0.1, (now - lastFrame) / 1000);
    lastFrame = now;
    if (camMode === "free" || !camInit || !state) return;
    // look at the leader and the followers in its formation (drones that left or went down are not followed)
    const all = [...drones.values()];
    const lead = state.master != null ? drones.get(state.master) : null;
    let use = lead ? all.filter((e) => e === lead || (e.d.role === "FOLLOWER" && e.d.master === lead.id)) : [];
    if (!use.length) use = all.filter((e) => e.d.role !== "DOWN" && e.d.role !== "RETIRED");
    if (!use.length) use = all.filter((e) => e.d.role !== "DOWN");
    if (!use.length) use = all;
    if (!use.length) return;
    C.Cartesian3.fromElements(0, 0, 0, centre);
    const pts = use.map((e) => posNow(e, now));
    pts.forEach((p) => C.Cartesian3.add(centre, p, centre));
    C.Cartesian3.divideByScalar(centre, pts.length, centre);
    let extent = 0;
    for (const p of pts) extent = Math.max(extent, C.Cartesian3.distance(p, centre));
    extent = Math.min(extent, 600);
    const leadHdg = lead ? lerpAngle(lead.h0, lead.h1, frac(lead, now)) : (camHeading ?? 0);
    let want, pitch, range;
    if (camMode === "chase") { want = leadHdg; pitch = -24; range = extent * 2.1 + 55; }
    else if (camMode === "orbit") { want = (camHeading ?? leadHdg) + dt * 9; pitch = -28; range = extent * 2.2 + 70; }
    else { want = leadHdg; pitch = -89.5; range = extent * 3.2 + 120; }
    camHeading = camHeading == null ? want : lerpAngle(camHeading, want, camMode === "orbit" ? 1 : Math.min(1, dt * 1.2));
    // glide in slowly from far away (the first seconds after loading) so the terrain tiles can arrive on the way
    camRange = camRange + (range - camRange) * Math.min(1, dt * (camRange > range * 3 ? 0.7 : 1.5));
    const p = camMode === "top" ? pitch : Math.max(-89, Math.min(-3, pitch + userPitch - lift));
    viewer.camera.lookAt(centre, new C.HeadingPitchRange(C.Math.toRadians(camHeading + userYaw), C.Math.toRadians(p),
                                                         camRange * zoomMul));
    const camPos = viewer.camera.positionCartographic, below = scene.globe.getHeight(camPos);
    if (below !== undefined && camPos.height < below + 12) lift = Math.min(lift + 60 * dt, 60);
    else lift = Math.max(lift - 10 * dt, 0);
  });

  // ------------------------------------------------------------------ stream and buttons
  let pending = false, latest = null;
  const es = new EventSource("/api/stream");
  es.onopen = () => { $("conn").textContent = "live"; $("conn").classList.add("live"); };
  es.onerror = () => { $("conn").textContent = "offline"; $("conn").classList.remove("live"); };
  es.onmessage = (ev) => {                            // draw only the newest snapshot, once per animation frame
    latest = JSON.parse(ev.data);
    if (pending) return;
    pending = true;
    requestAnimationFrame(() => { pending = false; update(latest); });
  };
  document.querySelectorAll(".speeds button").forEach((b) => b.addEventListener("click", () => cmd({ cmd: "speed", value: Number(b.dataset.speed) })));
  document.querySelectorAll("[data-fault]").forEach((b) => b.addEventListener("click", () =>
    cmd({ cmd: "fault", type: b.dataset.fault, target: b.dataset.target })));
  $("split").addEventListener("click", () => cmd({ cmd: "partition" }));
  $("heal").addEventListener("click", () => cmd({ cmd: "heal" }));
  $("pause").addEventListener("click", () => cmd({ cmd: state && state.running ? "pause" : "resume" }));
  $("hidepanel").addEventListener("click", () => {
    const hud = $("hud"), hide = !hud.classList.contains("collapsed");
    hud.classList.toggle("collapsed", hide);
    $("hidepanel").textContent = hide ? "Show panel" : "Hide panel";
    $("hidepanel").setAttribute("aria-expanded", String(!hide));
  });
})();
