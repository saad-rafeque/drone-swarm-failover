(() => {
  "use strict";
  const $ = (id) => document.getElementById(id);
  const COLORS = { MASTER: "#4a92ea", FOLLOWER: "#e8eef3", RETIRED: "#e0a33a", DOWN: "#ef6a5f",
                   transit: "#37b3a8", orphan: "#a88ae6", emergency: "#ff8a3d" };
  const DOING = {
    formation: "In formation", transit: "Repositioning (6 m below)", orphan: "Not heard by master (8 m above)",
    hover_no_master: "Hovering, electing master", cruise: "Leading to target", hold: "Hovering at target",
    takeoff: "Taking off", land: "Landing", idle: "On ground", climb_to_join: "Climbing to join",
    retire_climb: "Leaving formation", retire_transit: "Flying home", retire_land: "Landing at home",
    retired_on_ground: "Landed", emergency_drop: "Emergency descent", emergency_land: "Emergency landing",
    falling: "Falling", crashed: "Crashed", charging: "Charging (battery swap)",
  };
  const OFF_FORMATION = new Set(["transit", "orphan", "climb_to_join", "retire_climb", "retire_transit", "retire_land",
                                  "emergency_drop", "emergency_land"]);
  const PHASE = { IDLE: "On ground", TAKEOFF: "Takeoff", CRUISE: "Cruise", HOLD: "At target", LAND: "Landing", LANDED: "Landed",
                  CHARGE: "Charging stop" };
  const LEG_FRACTION = 0.55;      // same as the backend: a leg between charging stops uses ~half a battery
  const HISTORY_S = 240;
  const FOLLOW_ZOOM = 19;          // closest zoom when following: ~0.25 m per pixel, 10 m slots ~40 px apart
  let zoomToSwarm = true;

  let state = null, drawPending = false, lastT = -1, lastEventSeq = -1, paramsKey = "", pickWhat = null;
  let lastPan = 0, lastChart = 0, fitted = false;
  const selected = new Set();
  const drones = new Map();       // id -> {marker, trail, key}
  const rowEls = new Map();       // id -> cached table cells
  const history = new Map();      // id -> [[t, alt, color], ...]

  // ---------------------------------------------------------------- map
  const map = L.map("map", { zoomControl: true });
  const satellite = L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    { maxZoom: 19, attribution: "Imagery &copy; Esri, Maxar, Earthstar Geographics" });
  const streets = L.tileLayer("https://tile.openstreetmap.org/{z}/{x}/{y}.png",
    { maxZoom: 19, attribution: "&copy; OpenStreetMap contributors" });
  satellite.addTo(map);
  const layers = L.control.layers({ "Satellite (Esri)": satellite, Streets: streets }, null, { position: "topright" }).addTo(map);
  // Mapbox satellite when the user has put a token in config/map_keys.local.yaml (attribution and logo per Mapbox terms)
  const mapboxLogo = L.control({ position: "bottomleft" });
  mapboxLogo.onAdd = () => {
    const a = L.DomUtil.create("a", "mapbox-logo");
    a.href = "https://www.mapbox.com/about/maps"; a.target = "_blank"; a.rel = "noopener";
    a.innerHTML = '<img src="/static/mapbox-logo.svg" width="88" height="23" alt="Mapbox">';
    return a;
  };
  const getConfig = (tries = 5) => fetch("/api/config").then((r) => r.json())
    .catch((e) => (tries > 1 ? new Promise((ok) => setTimeout(ok, 1000)).then(() => getConfig(tries - 1)) : Promise.reject(e)));
  getConfig().then((cfg) => {
    if (!cfg.mapbox_token) return;
    const mb = L.tileLayer(`https://api.mapbox.com/v4/mapbox.satellite/{z}/{x}/{y}@2x.jpg90?access_token=${encodeURIComponent(cfg.mapbox_token)}`, {
      maxZoom: 21, maxNativeZoom: 20,
      attribution: '&copy; <a href="https://www.mapbox.com/about/maps" target="_blank" rel="noopener">Mapbox</a> ' +
        '&copy; <a href="http://www.openstreetmap.org/copyright" target="_blank" rel="noopener">OpenStreetMap</a> ' +
        '&copy; <a href="https://www.maxar.com/" target="_blank" rel="noopener">Maxar</a> ' +
        '<strong><a href="https://apps.mapbox.com/feedback/" target="_blank" rel="noopener">Improve this map</a></strong>' });
    layers.addBaseLayer(mb, "Satellite (Mapbox)");
    map.removeLayer(satellite); mb.addTo(map); mapboxLogo.addTo(map);
    map.on("baselayerchange", (e) => { if (e.layer === mb) mapboxLogo.addTo(map); else mapboxLogo.remove(); });
  }).catch((e) => console.error("Mapbox layer:", e));
  L.control.scale({ imperial: false }).addTo(map);
  map.setView([33.71, 73.03], 14);
  const zoomClass = () => {
    const c = map.getContainer().classList, z = map.getZoom();
    c.toggle("farzoom", z < 17); c.toggle("veryfar", z < 15);
  };
  map.on("zoomend", zoomClass); zoomClass();
  const pin = (cls, text) => L.divIcon({ className: "", html: `<div class="pin ${cls}">${text}</div>`, iconSize: [64, 24], iconAnchor: [20, 30] });
  const homeM = L.marker([0, 0], { icon: pin("home", "HOME"), interactive: false, keyboard: false }).addTo(map);
  const tgtM = L.marker([0, 0], { icon: pin("target", "TARGET"), interactive: false, keyboard: false }).addTo(map);
  const routeL = L.polyline([], { color: "#ffffff", weight: 2, opacity: 0.75, dashArray: "6 7" }).addTo(map);
  const trailRenderer = L.canvas({ padding: 0.3 });   // one canvas for all trails (100 drones stay smooth)
  const obstacleRenderer = L.canvas({ padding: 0.5 });
  const obstacleLayer = L.layerGroup().addTo(map);
  const plannedL = L.polyline([], { color: "#ffd166", weight: 3, opacity: 0.9 }).addTo(map);
  let obstaclesVersion = -1, routeKey = "", stopsKey = "";
  let obstacleData = { polys: [], circles: [] };
  const stopsLayer = L.layerGroup().addTo(map);
  function drawObstacles() {          // long routes carry tens of thousands of buildings: draw only those in view
    obstacleLayer.clearLayers();
    if (map.getZoom() < 13) return;
    const b = map.getBounds().pad(0.5);
    for (const o of obstacleData.polys) {
      if (o.s > b.getNorth() || o.n < b.getSouth() || o.w > b.getEast() || o.e < b.getWest()) continue;
      const wood = o.kind === "wood";
      L.polygon(o.pts, { renderer: obstacleRenderer, interactive: false, weight: 1,
        color: wood ? "#5fd39a" : "#ff8f7a", fillColor: wood ? "#3fbf86" : "#ef6a5f", fillOpacity: wood ? 0.25 : 0.4 })
        .addTo(obstacleLayer);
    }
    for (const [lat, lon, r] of obstacleData.circles) {
      if (!b.contains([lat, lon])) continue;
      L.circle([lat, lon], { radius: r, renderer: obstacleRenderer, interactive: false,
        weight: 1, color: "#5fd39a", fillColor: "#3fbf86", fillOpacity: 0.4 }).addTo(obstacleLayer);
    }
  }
  map.on("moveend", drawObstacles);
  async function loadObstacles(version) {
    obstaclesVersion = version;
    try {
      const o = await (await fetch("/api/obstacles")).json();
      obstacleData = {
        polys: o.polygons.map((pts, k) => {
          const lats = pts.map((q) => q[0]), lons = pts.map((q) => q[1]);
          return { pts, kind: o.kinds[k], s: Math.min(...lats), n: Math.max(...lats), w: Math.min(...lons), e: Math.max(...lons) };
        }),
        circles: o.circles,
      };
      drawObstacles();
    } catch (e) { obstaclesVersion = -1; }
  }

  function droneColor(d) {
    if (d.role === "DOWN") return COLORS.DOWN;
    if (d.status === "transit") return COLORS.transit;
    if (d.status === "orphan") return COLORS.orphan;
    if (d.status && d.status.startsWith("emergency")) return COLORS.emergency;
    return COLORS[d.role] || COLORS.FOLLOWER;
  }
  function droneSvg(color, down) {
    if (down) return `<svg viewBox="-20 -20 40 40" width="34" height="34" aria-hidden="true"><path d="M-9 -9L9 9M9 -9L-9 9" stroke="${color}" stroke-width="4.5" stroke-linecap="round"/></svg>`;
    const rotor = (x, y) => `<circle cx="${x}" cy="${y}" r="6.5" fill="${color}" fill-opacity="0.3" stroke="${color}" stroke-width="1.7"/>`;
    return `<svg viewBox="-20 -20 40 40" width="34" height="34" aria-hidden="true">
      <line x1="-11" y1="-11" x2="11" y2="11" stroke="#111a21" stroke-width="3.4" stroke-linecap="round"/>
      <line x1="11" y1="-11" x2="-11" y2="11" stroke="#111a21" stroke-width="3.4" stroke-linecap="round"/>
      ${rotor(-11, -11)}${rotor(11, -11)}${rotor(-11, 11)}${rotor(11, 11)}
      <rect x="-5" y="-6.5" width="10" height="13" rx="3.2" fill="${color}" stroke="#111a21" stroke-width="1.5"/>
      <path d="M0 -13L3.6 -7.6L-3.6 -7.6Z" fill="${color}" stroke="#111a21" stroke-width="1.1"/></svg>`;
  }
  function droneIcon(d, color) {
    return L.divIcon({ className: "drone-icon", iconSize: [34, 34], iconAnchor: [17, 17],
      html: `<div class="rot">${droneSvg(color, d.role === "DOWN")}</div><span class="tag"></span>` });
  }

  function updateMap(s) {
    const home = s.params.home, tgt = s.params.target;
    homeM.setLatLng(home); tgtM.setLatLng(tgt); routeL.setLatLngs([home, tgt]);
    if (s.obstacles_version !== obstaclesVersion) loadObstacles(s.obstacles_version);
    const rk = s.route ? `${s.route.length}:${s.route[0]}:${s.route[s.route.length - 1]}` : "";
    if (rk !== routeKey) { plannedL.setLatLngs(s.route || []); routeKey = rk; routeL.setStyle({ opacity: s.route ? 0.3 : 0.75 }); }
    const sk = `${(s.stops || []).length}:${s.next_stop}:${rk}`;
    if (sk !== stopsKey) {
      stopsKey = sk;
      stopsLayer.clearLayers();
      (s.stops || []).forEach(([lat, lon], k) => L.marker([lat, lon], { interactive: false, keyboard: false,
        icon: L.divIcon({ className: "", iconSize: [22, 22], iconAnchor: [11, 11],
          html: `<div class="stop${s.next_stop != null && k < s.next_stop ? " done" : ""}" title="Charging stop ${k + 1}">&#9889;</div>` }) })
        .addTo(stopsLayer));
    }
    if (!fitted) { map.fitBounds(L.latLngBounds([home, tgt]).pad(0.25)); fitted = true; }
    const showTrails = $("trails").checked;
    const many = s.drones.length > 30;   // big swarms: trail only the leader and drones that left formation
    let cx = 0, cy = 0, cn = 0, s0 = 90, n0 = -90, w0 = 180, e0 = -180;
    for (const d of s.drones) {
      const color = droneColor(d), key = color + d.role;
      let e = drones.get(d.id);
      if (!e) {
        e = { marker: L.marker([d.lat, d.lon], { icon: droneIcon(d, color), keyboard: false }).addTo(map),
              trail: L.polyline([], { color, weight: 2, opacity: 0.55, renderer: trailRenderer }).addTo(map), key };
        drones.set(d.id, e);
      } else if (e.key !== key) {
        e.marker.setIcon(droneIcon(d, color)); e.trail.setStyle({ color }); e.key = key;
      }
      e.marker.setLatLng([d.lat, d.lon]);
      const el = e.marker.getElement();
      if (el) {
        const rot = el.querySelector(".rot");
        if (rot && d.role !== "DOWN") rot.style.transform = `rotate(${d.hdg}deg)`;
        const tag = el.querySelector(".tag");
        if (tag) tag.textContent = `${d.id} · ${d.alt.toFixed(0)} m`;
      }
      if (d.role === "MASTER" || OFF_FORMATION.has(d.status)) e.keepTrail = true;
      if (showTrails && d.role !== "DOWN" && (!many || e.keepTrail)) {
        const pts = e.trail.getLatLngs(), last = pts[pts.length - 1];
        if (!last || Math.abs(last.lat - d.lat) + Math.abs(last.lng - d.lon) > 2e-6) {
          e.trail.addLatLng([d.lat, d.lon]);
          if (pts.length > 900) e.trail.setLatLngs(pts.slice(-800));
        }
      }
      if (d.role !== "DOWN" && !d.landed) {
        cx += d.lat; cy += d.lon; cn++;
        s0 = Math.min(s0, d.lat); n0 = Math.max(n0, d.lat); w0 = Math.min(w0, d.lon); e0 = Math.max(e0, d.lon);
      }
    }
    if (!showTrails) drones.forEach((e) => e.trail.setLatLngs([]));
    const now = performance.now();
    if ($("follow").checked && cn && now - lastPan > 500 && !pickWhat) {
      if (zoomToSwarm) { map.fitBounds([[s0, w0], [n0, e0]], { maxZoom: FOLLOW_ZOOM, padding: [60, 60] }); zoomToSwarm = false; }
      else map.panTo([cx / cn, cy / cn], { animate: true, duration: 0.4 });
      lastPan = now;
    }
  }

  // ---------------------------------------------------------------- side panel
  function fmtTime(sec) {
    if (sec == null) return "—";
    const m = Math.floor(sec / 60), s = Math.round(sec % 60);
    return m >= 60 ? `${Math.floor(m / 60)} h ${m % 60} min` : `${m}:${String(s).padStart(2, "0")}`;
  }
  function haversine(a, b) {
    const R = 6371000, r = Math.PI / 180, dLat = (b[0] - a[0]) * r, dLon = (b[1] - a[1]) * r;
    const h = Math.sin(dLat / 2) ** 2 + Math.cos(a[0] * r) * Math.cos(b[0] * r) * Math.sin(dLon / 2) ** 2;
    return 2 * R * Math.asin(Math.sqrt(h));
  }
  function routeHint() {
    const h = [Number($("home_lat").value), Number($("home_lon").value)], t = [Number($("tgt_lat").value), Number($("tgt_lon").value)];
    if (h.some(isNaN) || t.some(isNaN)) return;
    const dist = haversine(h, t), v = Number($("cruise").value) || 5, e = Number($("endurance").value) || 25;
    const need = dist / v / 60;
    const leg = Math.max(1000, LEG_FRACTION * e * 60 * v), stops = dist > leg + 500 ? Math.ceil((dist - 500) / leg) - 1 : 0;
    const fmtMin = (m) => (m >= 90 ? `${(m / 60).toFixed(1)} h` : `${m.toFixed(0)} min`);
    $("route").textContent = `Route ${(dist / 1000).toFixed(dist > 20000 ? 0 : 2)} km · about ${fmtMin(need)} at ${v} m/s · battery ${e} min` +
      (stops ? ` · about ${stops} charging stops, one every ${(leg / 1000).toFixed(1)} km` : "") +
      (dist > 400000 ? " — too far: the page accepts up to 400 km" : "");
    homeM.setLatLng(h); tgtM.setLatLng(t); routeL.setLatLngs([h, t]);
  }
  function fillParams(p) {
    const key = JSON.stringify(p);
    if (key === paramsKey) return;
    paramsKey = key;
    const set = (id, v) => { if (document.activeElement !== $(id)) $(id).value = v; };
    set("home_lat", p.home[0]); set("home_lon", p.home[1]); set("tgt_lat", p.target[0]); set("tgt_lon", p.target[1]);
    set("n", p.n); set("cruise", p.cruise_mps); set("endurance", p.endurance_min); set("seed", p.seed);
    set("obstacles", p.obstacles || "none"); set("avoider", p.avoider || "apf"); set("altitude", p.altitude || "auto");
    set("shape", p.shape || "V");
    routeHint();
  }
  ["home_lat", "home_lon", "tgt_lat", "tgt_lon", "cruise", "endurance"].forEach((id) => $(id).addEventListener("input", routeHint));

  function buildRows(s) {
    const tb = $("rows");
    if (rowEls.size === s.drones.length) return;
    tb.innerHTML = ""; rowEls.clear();
    for (const d of s.drones) {
      const tr = document.createElement("tr");
      tr.dataset.id = d.id;
      tr.innerHTML = `<td><input type="checkbox" aria-label="Select drone ${d.id}"></td>
        <td><span class="idcell"><span class="sw"></span>${d.id}</span></td><td class="role"></td>
        <td class="num alt"></td><td class="num spd"></td><td><span class="batt"><i><b></b></i><span></span></span></td><td class="doing"></td>`;
      tr.querySelector("input").addEventListener("change", (ev) => {
        ev.target.checked ? selected.add(d.id) : selected.delete(d.id);
        tr.classList.toggle("sel", ev.target.checked);
      });
      tb.appendChild(tr);
      const q = (sel) => tr.querySelector(sel);
      rowEls.set(d.id, { tr, sw: q(".sw"), role: q(".role"), alt: q(".alt"), spd: q(".spd"), bar: q(".batt b"),
                         pct: q(".batt span"), doing: q(".doing") });
    }
  }
  function updateRows(s) {
    buildRows(s);
    for (const d of s.drones) {
      const r = rowEls.get(d.id);
      if (!r) continue;
      r.tr.classList.toggle("down", d.role === "DOWN");
      r.sw.style.background = droneColor(d);
      const flags = (d.gps_ok ? "" : " · no GPS") + (d.radio_ok ? "" : " · radio cut");
      r.role.textContent = d.role === "MASTER" ? "Master" : d.role === "RETIRED" ? "Left" : d.role === "DOWN" ? "Down" : "Follower";
      r.alt.textContent = d.alt.toFixed(1);
      r.spd.textContent = d.speed.toFixed(1);
      r.bar.style.width = `${Math.max(0, Math.min(100, d.battery))}%`;
      r.bar.style.background = d.battery <= 10 ? "var(--alert)" : d.battery <= 30 ? "var(--warn)" : "var(--ok)";
      r.pct.textContent = `${d.battery.toFixed(0)}%`;
      r.doing.textContent = (DOING[d.status] || d.status) + flags;
    }
  }

  function updateStatus(s) {
    $("clock").textContent = `t = ${s.t.toFixed(0)} s`;
    $("actual").textContent = s.running ? `running ${s.actual_speed}×` : "paused";
    $("pause").textContent = s.running ? "Pause" : "Resume";
    document.querySelectorAll(".speeds button").forEach((b) => b.setAttribute("aria-pressed", String(Number(b.dataset.speed) === s.speed)));
    $("r_phase").textContent = s.phase ? (PHASE[s.phase] || s.phase) : "No master";
    $("r_master").innerHTML = s.masters.length > 1 ? `Split: ${s.masters.join(", ")}` :
      s.master ? `Drone ${s.master} <small>term ${s.term}</small>` : "electing…";
    $("r_alive").textContent = `${s.alive} / ${s.total}`;
    const rms = $("r_rms");
    rms.textContent = s.rms == null ? "—" : `${s.rms.toFixed(2)} m`;
    rms.className = "v " + (s.rms == null ? "" : s.rms < 2 ? "ok" : "bad");
    const sep = $("r_sep");
    sep.textContent = s.min_sep == null ? "—" : `${s.min_sep.toFixed(1)} m`;
    sep.className = "v " + (s.min_sep == null ? "" : s.min_sep >= 5 ? "ok" : "bad");
    const stopTxt = s.stops && s.stops.length ? ` · stop ${Math.min((s.next_stop || 0) + 1, s.stops.length)}/${s.stops.length}` : "";
    $("r_left").innerHTML = s.dist_left == null ? "—" : `${(s.dist_left / 1000).toFixed(s.dist_left > 20000 ? 0 : 2)} km <small>ETA ${fmtTime(s.eta)}${stopTxt}</small>`;
    const hits = $("r_hits");
    hits.textContent = s.route ? String(s.hits) : "—";
    hits.className = "v " + (s.route ? (s.hits ? "bad" : "ok") : "");
    $("r_avoid").textContent = s.route ? s.avoiders[s.params.avoider] : "No obstacles (open sky)";
    if (s.loading) $("route").textContent = s.loading + "…";
  }

  function updateEvents(s) {
    const evs = s.events;
    const top = evs.length ? evs[evs.length - 1][0] : -1;
    if (top === lastEventSeq) return;
    lastEventSeq = top;
    $("events").innerHTML = evs.slice().reverse().map(([, t, kind, text]) =>
      `<li class="${kind}"><span class="t">${fmtTime(t)}</span><span class="m"></span></li>`).join("");
    [...$("events").querySelectorAll(".m")].forEach((el, k) => { el.textContent = evs[evs.length - 1 - k][3]; });
  }

  function updateHistory(s) {
    if (s.t < lastT - 0.5) { history.clear(); drones.forEach((e) => { e.trail.setLatLngs([]); }); }
    lastT = s.t;
    for (const d of s.drones) {
      if (!history.has(d.id)) history.set(d.id, []);
      const h = history.get(d.id);
      if (!h.length || s.t - h[h.length - 1][0] >= 0.5) h.push([s.t, d.alt, droneColor(d)]);
      while (h.length && s.t - h[0][0] > HISTORY_S) h.shift();
    }
  }
  function drawChart(s) {
    const c = $("altchart"), dpr = window.devicePixelRatio || 1, w = c.clientWidth, h = 150;
    c.width = Math.round(w * dpr); c.height = Math.round(h * dpr);
    const ctx = c.getContext("2d");
    ctx.scale(dpr, dpr);
    const L = 30, R = w - 6, T = 8, B = h - 18, top = Math.max(s.fence_alt, 10);
    const X = (t) => L + ((t - (s.t - HISTORY_S)) / HISTORY_S) * (R - L), Y = (a) => B - (a / top) * (B - T);
    ctx.font = "11px IBM Plex Mono, monospace"; ctx.fillStyle = "#71828e"; ctx.strokeStyle = "#273540"; ctx.lineWidth = 1;
    [[0, "0"], [s.cruise_alt, `${s.cruise_alt}`], [s.fence_alt, `${s.fence_alt}`]].forEach(([a, lab]) => {
      ctx.beginPath(); ctx.moveTo(L, Y(a)); ctx.lineTo(R, Y(a)); ctx.stroke(); ctx.fillText(lab, 4, Y(a) + 4);
    });
    ctx.fillText("m", 4, T + 4);
    ctx.fillText(`last ${HISTORY_S / 60} min`, R - 78, h - 4);
    ctx.lineWidth = 1.6;
    history.forEach((pts) => {          // one stroke per run of the same colour
      let k = 1;
      while (k < pts.length) {
        const color = pts[k][2];
        ctx.strokeStyle = color; ctx.beginPath(); ctx.moveTo(X(pts[k - 1][0]), Y(pts[k - 1][1]));
        while (k < pts.length && pts[k][2] === color) { ctx.lineTo(X(pts[k][0]), Y(pts[k][1])); k++; }
        ctx.stroke();
      }
    });
  }

  function render() {
    drawPending = false;
    const s = state;
    if (!s) return;
    $("backend").textContent = s.backend;
    fillParams(s.params);
    updateHistory(s);
    updateMap(s);
    updateRows(s);
    updateStatus(s);
    updateEvents(s);
    const now = performance.now();
    if (now - lastChart > 250) { drawChart(s); lastChart = now; }
  }

  // ---------------------------------------------------------------- commands
  let toastTimer = 0;
  function toast(msg) {
    const t = $("toast"); t.textContent = msg; t.hidden = false;
    clearTimeout(toastTimer); toastTimer = setTimeout(() => { t.hidden = true; }, 2600);
  }
  async function cmd(body) {
    try {
      const r = await fetch("/api/cmd", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
      const j = await r.json();
      toast(j.msg || (j.ok ? "Done" : "Failed"));
      return j;
    } catch (e) { toast("Ground control is not responding (is scripts/gcs.py running?)"); return { ok: false }; }
  }
  function clearSwarm() {
    selected.clear(); rowEls.clear(); $("rows").innerHTML = ""; $("sel_all").checked = false;
    drones.forEach((e) => { map.removeLayer(e.marker); map.removeLayer(e.trail); }); drones.clear(); history.clear();
  }
  $("start").addEventListener("click", async () => {
    const r = await cmd({ cmd: "start", n: Number($("n").value), cruise_mps: Number($("cruise").value),
      endurance_min: Number($("endurance").value), seed: Number($("seed").value),
      obstacles: $("obstacles").value, avoider: $("avoider").value, altitude: $("altitude").value,
      shape: $("shape").value,
      home: [Number($("home_lat").value), Number($("home_lon").value)], target: [Number($("tgt_lat").value), Number($("tgt_lon").value)] });
    if (r.ok) { fitted = false; zoomToSwarm = true; clearSwarm(); }
  });
  $("reset").addEventListener("click", async () => {
    const r = await cmd({ cmd: "reset" });
    if (r.ok) { zoomToSwarm = true; clearSwarm(); }
  });
  $("pause").addEventListener("click", () => cmd({ cmd: state && state.running ? "pause" : "resume" }));
  document.querySelectorAll(".speeds button").forEach((b) => b.addEventListener("click", () => cmd({ cmd: "speed", value: Number(b.dataset.speed) })));
  document.querySelectorAll("[data-quick]").forEach((b) => b.addEventListener("click", () => cmd({ cmd: "fault", type: "crash", target: b.dataset.quick })));
  $("partition").addEventListener("click", () => cmd({ cmd: "partition" }));
  $("heal").addEventListener("click", () => cmd({ cmd: "heal" }));
  $("apply").addEventListener("click", () => {
    if (!selected.size) { toast("Tick one or more drones in the table first"); return; }
    cmd({ cmd: "fault", type: $("fault_type").value, target: [...selected] });
  });
  $("sel_all").addEventListener("change", (ev) => {
    $("rows").querySelectorAll("tr").forEach((tr) => {
      const box = tr.querySelector("input"); box.checked = ev.target.checked; box.dispatchEvent(new Event("change"));
    });
  });
  $("follow").addEventListener("change", (ev) => { if (ev.target.checked) zoomToSwarm = true; });
  const startPick = (what) => { pickWhat = what; $("pickwhat").textContent = what; $("pickhint").hidden = false; map.getContainer().style.cursor = "crosshair"; };
  $("pick_home").addEventListener("click", () => startPick("home"));
  $("pick_target").addEventListener("click", () => startPick("target"));
  map.on("click", (ev) => {
    if (!pickWhat) return;
    const lat = ev.latlng.lat.toFixed(6), lon = ev.latlng.lng.toFixed(6);
    if (pickWhat === "home") { $("home_lat").value = lat; $("home_lon").value = lon; } else { $("tgt_lat").value = lat; $("tgt_lon").value = lon; }
    pickWhat = null; $("pickhint").hidden = true; map.getContainer().style.cursor = "";
    routeHint();
    toast("Press Start mission to fly this route");
  });

  // ---------------------------------------------------------------- live stream
  let faultsFilled = false;
  function connect() {
    const es = new EventSource("/api/stream");
    es.onopen = () => { $("conn").textContent = "live"; $("conn").classList.add("live"); };
    es.onerror = () => { $("conn").textContent = "offline"; $("conn").classList.remove("live"); };
    es.onmessage = (ev) => {
      state = JSON.parse(ev.data);
      if (!faultsFilled) {
        $("fault_type").innerHTML = Object.entries(state.faults).map(([k, v]) => `<option value="${k}">${v}</option>`).join("");
        $("avoider").innerHTML = Object.entries(state.avoiders).map(([k, v]) => {
          const off = k.startsWith("rl") && !state.policy_available;
          return `<option value="${k}"${off ? " disabled" : ""}>${v}${off ? " (not trained yet)" : ""}</option>`;
        }).join("");
        $("avoider").value = state.params.avoider || "apf";
        $("altitude").innerHTML = Object.entries(state.altitudes).map(([k, v]) => `<option value="${k}">${v}</option>`).join("");
        $("altitude").value = state.params.altitude || "auto";
        $("shape").innerHTML = Object.entries(state.shapes).map(([k, v]) => `<option value="${k}">${v}</option>`).join("");
        $("shape").value = state.params.shape || "V";
        faultsFilled = true;
      }
      if (!drawPending) { drawPending = true; requestAnimationFrame(render); }
    };
  }
  connect();
})();
