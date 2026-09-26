// 3-D view of the running mission: Cesium terrain, 3-D buildings, the drones at their real heights above the
// ground, the planned route and the obstacles the simulator uses. Reads the same /api/stream as the Mission page.
(async () => {
  "use strict";
  const $ = (id) => document.getElementById(id);
  const COLORS = { MASTER: "#4a92ea", FOLLOWER: "#e8eef3", RETIRED: "#e0a33a", DOWN: "#ef6a5f" };
  const PHASE = { IDLE: "On ground", TAKEOFF: "Takeoff", CRUISE: "Cruise", HOLD: "At target", LAND: "Landing",
                  LANDED: "Landed", CHARGE: "Charging stop" };
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
  Cesium.Ion.defaultAccessToken = keys.cesium_ion_token;

  const viewer = new Cesium.Viewer("cesium", {
    terrain: Cesium.Terrain.fromWorldTerrain(),
    animation: false, timeline: false, geocoder: false, homeButton: false, sceneModePicker: false,
    navigationHelpButton: false, fullscreenButton: false, infoBox: false, selectionIndicator: false,
    baseLayerPicker: false,
  });
  viewer.scene.globe.depthTestAgainstTerrain = true;
  if (keys.mapbox_token) {   // sharper imagery with the user's Mapbox token (Cesium's default imagery otherwise)
    viewer.imageryLayers.removeAll();
    viewer.imageryLayers.addImageryProvider(new Cesium.UrlTemplateImageryProvider({
      url: `https://api.mapbox.com/v4/mapbox.satellite/{z}/{x}/{y}@2x.jpg90?access_token=${encodeURIComponent(keys.mapbox_token)}`,
      maximumLevel: 20, credit: new Cesium.Credit("© Mapbox © OpenStreetMap © Maxar", true) }));
  }
  let osmBuildings = null, photo = null;
  try { osmBuildings = await Cesium.createOsmBuildingsAsync(); viewer.scene.primitives.add(osmBuildings); }
  catch (e) { toast("3-D buildings could not load"); }
  $("buildings").addEventListener("change", (ev) => { if (osmBuildings) osmBuildings.show = ev.target.checked; });
  $("photo").addEventListener("change", async (ev) => {
    if (ev.target.checked && !photo) {
      try { photo = await Cesium.createGooglePhotorealistic3DTileset(); viewer.scene.primitives.add(photo); }
      catch (e) { toast("Photorealistic tiles are not available for this token"); ev.target.checked = false; return; }
    }
    if (photo) photo.show = ev.target.checked;
    viewer.scene.globe.show = !ev.target.checked;
    if (osmBuildings) osmBuildings.show = !ev.target.checked && $("buildings").checked;
  });

  const quad = (color, down) => {
    const body = down
      ? `<path d="M-9 -9L9 9M9 -9L-9 9" stroke="${color}" stroke-width="4.5" stroke-linecap="round"/>`
      : `<line x1="-11" y1="-11" x2="11" y2="11" stroke="#111a21" stroke-width="3.4"/><line x1="11" y1="-11" x2="-11" y2="11" stroke="#111a21" stroke-width="3.4"/>` +
        [[-11, -11], [11, -11], [-11, 11], [11, 11]].map(([x, y]) => `<circle cx="${x}" cy="${y}" r="6.5" fill="${color}" fill-opacity="0.35" stroke="${color}" stroke-width="1.8"/>`).join("") +
        `<rect x="-5" y="-6.5" width="10" height="13" rx="3.2" fill="${color}" stroke="#111a21" stroke-width="1.5"/>`;
    return "data:image/svg+xml;charset=utf-8," + encodeURIComponent(`<svg xmlns="http://www.w3.org/2000/svg" viewBox="-20 -20 40 40" width="48" height="48">${body}</svg>`);
  };
  const icons = Object.fromEntries(Object.entries(COLORS).map(([k, c]) => [k, quad(c, k === "DOWN")]));
  const REL = Cesium.HeightReference.RELATIVE_TO_GROUND;
  const drones = new Map();
  const routeEnt = viewer.entities.add({ polyline: { positions: [], width: 4, clampToGround: true,
    material: Cesium.Color.fromCssColorString("#ffd166") } });
  const pins = {};
  const pin = (name, text, color) => viewer.entities.add({ name, position: Cesium.Cartesian3.fromDegrees(0, 0),
    point: { pixelSize: 10, color: Cesium.Color.fromCssColorString(color), outlineColor: Cesium.Color.WHITE, outlineWidth: 2,
             heightReference: Cesium.HeightReference.CLAMP_TO_GROUND, disableDepthTestDistance: Number.POSITIVE_INFINITY },
    label: { text, font: "600 14px Barlow Condensed, sans-serif", fillColor: Cesium.Color.WHITE, showBackground: true,
             backgroundColor: Cesium.Color.fromCssColorString(color).withAlpha(0.85), pixelOffset: new Cesium.Cartesian2(0, -22),
             heightReference: Cesium.HeightReference.CLAMP_TO_GROUND, disableDepthTestDistance: Number.POSITIVE_INFINITY } });
  pins.home = pin("home", "HOME", "#2d3b46");
  pins.target = pin("target", "TARGET", "#c0392b");

  const obstacleSource = new Cesium.CustomDataSource("obstacles");
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
        hierarchy: Cesium.Cartesian3.fromDegreesArray(o.poly.flatMap(([lat, lon]) => [lon, lat])),
        material: Cesium.Color.fromCssColorString(o.kind === "wood" ? "#3fbf86" : "#ef6a5f").withAlpha(0.45),
        classificationType: Cesium.ClassificationType.BOTH } });
    }
    drawnAt = center;
  }
  async function loadObstacles(v, center) {
    obstaclesVersion = v;
    const o = await (await fetch("/api/obstacles")).json();
    obstacleAll = o.polygons.map((poly, k) => ({ poly, kind: o.kinds[k], lat: poly[0][0], lon: poly[0][1] }));
    drawObstacles(center);
  }

  const stopSource = new Cesium.CustomDataSource("stops");
  viewer.dataSources.add(stopSource);
  let stopsKey = "";
  function drawStops(s) {
    const key = `${(s.stops || []).length}:${s.next_stop}`;
    if (key === stopsKey) return;
    stopsKey = key;
    stopSource.entities.removeAll();
    (s.stops || []).forEach(([lat, lon], k) => stopSource.entities.add({ position: Cesium.Cartesian3.fromDegrees(lon, lat),
      point: { pixelSize: 9, color: Cesium.Color.fromCssColorString(s.next_stop != null && k < s.next_stop ? "#6b7b87" : "#e0a33a"),
               outlineColor: Cesium.Color.WHITE, outlineWidth: 2, heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
               disableDepthTestDistance: Number.POSITIVE_INFINITY },
      label: { text: `Stop ${k + 1}`, font: "500 12px Barlow, sans-serif", pixelOffset: new Cesium.Cartesian2(0, -16),
               showBackground: true, heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
               distanceDisplayCondition: new Cesium.DistanceDisplayCondition(0, 5000),
               disableDepthTestDistance: Number.POSITIVE_INFINITY } }));
  }
  let flown = false, lastRouteKey = "", state = null;
  function update(s) {
    state = s;
    $("clock").textContent = `t = ${s.t.toFixed(0)} s`;
    $("r_phase").textContent = s.phase ? (PHASE[s.phase] || s.phase) : "No leader";
    $("r_master").textContent = s.master ? `Drone ${s.master}` : "electing…";
    $("r_alive").textContent = `${s.alive} / ${s.total}`;
    $("r_hits").textContent = s.route ? String(s.hits) : "—";
    $("pause").textContent = s.running ? "Pause" : "Resume";
    document.querySelectorAll(".speeds button").forEach((b) => b.setAttribute("aria-pressed", String(Number(b.dataset.speed) === s.speed)));
    const home = s.params.home, tgt = s.params.target;
    pins.home.position = Cesium.Cartesian3.fromDegrees(home[1], home[0]);
    pins.target.position = Cesium.Cartesian3.fromDegrees(tgt[1], tgt[0]);
    const rk = s.route ? `${s.route.length}:${s.route[0]}` : `${home}:${tgt}`;
    if (rk !== lastRouteKey) {
      lastRouteKey = rk;
      const pts = s.route || [home, tgt];
      routeEnt.polyline.positions = Cesium.Cartesian3.fromDegreesArray(pts.flatMap(([lat, lon]) => [lon, lat]));
    }
    const lead = s.drones.find((d) => d.id === s.master) || s.drones[0];
    const center = lead ? [lead.lat, lead.lon] : null;
    if (s.obstacles_version !== obstaclesVersion) loadObstacles(s.obstacles_version, center).catch(() => {});
    else if (obstacleAll.length > 3000 && center && (!drawnAt || Math.abs(center[0] - drawnAt[0]) + Math.abs(center[1] - drawnAt[1]) > NEAR_DEG / 2)) {
      drawObstacles(center);
    }
    drawStops(s);
    for (const d of s.drones) {
      let e = drones.get(d.id);
      const role = d.role in COLORS ? d.role : "FOLLOWER";
      const pos = Cesium.Cartesian3.fromDegrees(d.lon, d.lat, Math.max(d.alt, 0));
      if (!e) {
        const color = Cesium.Color.fromCssColorString(COLORS[role]);
        e = {
          marker: viewer.entities.add({ position: pos,
            billboard: { image: icons[role], scale: 0.75, heightReference: REL, disableDepthTestDistance: Number.POSITIVE_INFINITY },
            label: { text: "", font: "500 12px IBM Plex Mono, monospace", fillColor: Cesium.Color.WHITE, showBackground: true,
                     backgroundColor: Cesium.Color.fromCssColorString("#0a1015").withAlpha(0.75), pixelOffset: new Cesium.Cartesian2(26, -8),
                     heightReference: REL, disableDepthTestDistance: Number.POSITIVE_INFINITY,
                     distanceDisplayCondition: new Cesium.DistanceDisplayCondition(0, 1500) } }),
          stem: viewer.entities.add({ position: Cesium.Cartesian3.fromDegrees(d.lon, d.lat, Math.max(d.alt, 0.2) / 2),
            cylinder: { length: Math.max(d.alt, 0.2), topRadius: 0.25, bottomRadius: 0.25, heightReference: REL,
                        material: color.withAlpha(0.45) } }),
          role,
        };
        drones.set(d.id, e);
      }
      if (e.role !== role) {
        e.role = role;
        e.marker.billboard.image = icons[role];
        e.stem.cylinder.material = Cesium.Color.fromCssColorString(COLORS[role]).withAlpha(0.45);
      }
      e.marker.position = pos;
      e.marker.label.text = `${d.id} · ${d.alt.toFixed(0)} m`;
      e.stem.position = Cesium.Cartesian3.fromDegrees(d.lon, d.lat, Math.max(d.alt, 0.2) / 2);
      e.stem.cylinder.length = Math.max(d.alt, 0.2);
    }
    const m = s.drones.find((d) => d.id === s.master) || s.drones[0];
    if (!flown && m) {
      flown = true;
      viewer.camera.flyTo({ destination: Cesium.Cartesian3.fromDegrees(m.lon, m.lat - 0.0035, 320),
                            orientation: { heading: 0, pitch: Cesium.Math.toRadians(-32), roll: 0 }, duration: 2 });
    }
    if ($("follow").checked && m && drones.get(m.id)) {
      const tracked = drones.get(m.id).marker;
      if (viewer.trackedEntity !== tracked) viewer.trackedEntity = tracked;
    } else if (viewer.trackedEntity) {
      viewer.trackedEntity = undefined;
    }
  }

  let pending = false;
  const es = new EventSource("/api/stream");
  es.onopen = () => { $("conn").textContent = "live"; $("conn").classList.add("live"); };
  es.onerror = () => { $("conn").textContent = "offline"; $("conn").classList.remove("live"); };
  es.onmessage = (ev) => {
    const s = JSON.parse(ev.data);
    if (pending) { state = s; return; }
    pending = true;
    requestAnimationFrame(() => { pending = false; update(state && state.t > s.t ? state : s); });
  };
  document.querySelectorAll(".speeds button").forEach((b) => b.addEventListener("click", () => cmd({ cmd: "speed", value: Number(b.dataset.speed) })));
  $("kill").addEventListener("click", () => cmd({ cmd: "fault", type: "crash", target: "master" }));
  $("pause").addEventListener("click", () => cmd({ cmd: state && state.running ? "pause" : "resume" }));
})();
