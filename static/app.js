/**
 * Hudson River AR Broadcast & PTZ Mission Control Frontend (`static/app.js`)
 *
 * Supports three URL rendering modes from a single zero-build codebase:
 *   1. `/`          — Interactive Mission Control dashboard (live OBS backdrop + target queue + calibration)
 *   2. `/?obs=1`    — Fullscreen 16:9 simulated PTZ zoom broadcast view + lower-third chyron
 *   3. `/?hud=1`    — Zero-copy transparent 60fps AR HUD overlay for OBS Browser Source
 *                     (renders smooth C1-interpolated callsign pills & velocity vectors over 4K video)
 */
(() => {
  const params = new URLSearchParams(window.location.search);
  const isObsMode = params.get("obs") === "1";
  const isHudMode = params.get("hud") === "1";
  const showHudPtzBox = params.get("ptzbox") === "1";
  const autoWebcam = params.get("webcam") === "1";
  if (isObsMode) {
    document.body.classList.add("obs-mode");
  }
  if (isHudMode) {
    document.documentElement.classList.add("hud-mode");
    document.body.classList.add("hud-mode");
    if (params.get("chyron") === "0") {
      document.body.classList.add("no-chyron");
    }
  }

  const synCanvas = document.getElementById("synopticCanvas");
  const synCtx = synCanvas.getContext("2d");
  const ptzCanvas = document.getElementById("ptzCanvas");
  const ptzCtx = ptzCanvas.getContext("2d");
  const webcamVideo = document.getElementById("webcamVideo");

  const snapshotImg = new Image();
  snapshotImg.src = "/snapshot.jpg";

  let videoMode = "snapshot"; // "obs" | "snapshot" | "webcam"
  let obsBitmap = null;
  let obsAutoSwitched = false;
  let serverState = null;
  let lastPollTs = performance.now();

  let showZoomRings = false;
  let showLandmarks = false;
  let manualPointOverride = null; // {u, v, zoom_x, label}

  // Smooth spring-damper PTZ state in normalized synoptic coordinates
  const cam = {
    u: 0.5,
    v: 0.5,
    vu: 0.0,
    vv: 0.0,
    zoom: 1.0,
    vzoom: 0.0,
  };

  // --- UI Controls ---
  const videoSourceSelect = document.getElementById("videoSourceSelect");
  const obsCamInput = document.getElementById("obsCamInput");
  const btnConnectObs = document.getElementById("btnConnectObs");
  const cameraDeviceSelect = document.getElementById("cameraDeviceSelect");
  const btnAuto = document.getElementById("btnAuto");
  const btnNext = document.getElementById("btnNext");
  const cvModeSelect = document.getElementById("cvModeSelect");
  const btnSimShips = document.getElementById("btnSimShips");
  const btnRings = document.getElementById("btnRings");
  const btnLandmarks = document.getElementById("btnLandmarks");
  const btnSaveCalib = document.getElementById("btnSaveCalib");
  const silhouetteSelect = document.getElementById("silhouetteMode");

  async function postJson(url, data) {
    try {
      await fetch(url, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(data),
      });
    } catch (_) {}
  }

  async function startWebcam(deviceId = null) {
    try {
      const constraints = {
        video: {
          width: { ideal: 3840 },
          height: { ideal: 2160 },
          ...(deviceId ? { deviceId: { exact: deviceId } } : {}),
        },
        audio: false,
      };
      const stream = await navigator.mediaDevices.getUserMedia(constraints);
      webcamVideo.srcObject = stream;
      await webcamVideo.play();
      videoMode = "webcam";

      // Populate camera list (Insta360 Link 2C, OBS Virtual Camera, etc.)
      const devices = await navigator.mediaDevices.enumerateDevices();
      const cams = devices.filter((d) => d.kind === "videoinput");
      if (cams.length > 0) {
        cameraDeviceSelect.innerHTML = "";
        cams.forEach((c, idx) => {
          const opt = document.createElement("option");
          opt.value = c.deviceId;
          opt.textContent = c.label || `Camera ${idx + 1}`;
          if (deviceId && c.deviceId === deviceId) opt.selected = true;
          else if (!deviceId && /insta360|link 2c/i.test(c.label)) opt.selected = true;
          cameraDeviceSelect.appendChild(opt);
        });
        cameraDeviceSelect.style.display = "inline-block";
      }
    } catch (err) {
      alert("Could not open camera: " + err.message + "\nFalling back to Chelsea snapshot.");
      videoSourceSelect.value = "snapshot";
      videoMode = "snapshot";
    }
  }

  videoSourceSelect.addEventListener("change", () => {
    if (videoSourceSelect.value === "webcam") {
      startWebcam();
    } else {
      if (webcamVideo.srcObject) {
        webcamVideo.srcObject.getTracks().forEach((t) => t.stop());
        webcamVideo.srcObject = null;
      }
      cameraDeviceSelect.style.display = "none";
      videoMode = videoSourceSelect.value; // "obs" or "snapshot"
    }
  });

  function connectObsCamera() {
    const val = obsCamInput.value.trim();
    if (!val) return;
    videoSourceSelect.value = "obs";
    videoMode = "obs";
    postJson("/api/settings", { obs_url: val });
  }

  btnConnectObs.addEventListener("click", connectObsCamera);
  obsCamInput.addEventListener("keydown", (e) => {
    if (e.key === "Enter") connectObsCamera();
  });

  // Pipelined OBS preview frame fetcher (~6-18 fps on demand, zero MJPEG memory leak across Chrome/Safari)
  async function obsFrameLoop() {
    while (true) {
      if (serverState?.obs_cam?.connected && (videoMode === "obs" || !obsAutoSwitched)) {
        try {
          const resp = await fetch("/api/cam_frame.jpg?t=" + Date.now(), { cache: "no-store" });
          if (resp.ok) {
            const blob = await resp.blob();
            const bmp = await createImageBitmap(blob);
            if (obsBitmap) obsBitmap.close();
            obsBitmap = bmp;
            if (!obsAutoSwitched) {
              obsAutoSwitched = true;
              videoMode = "obs";
              videoSourceSelect.value = "obs";
            }
          }
        } catch (_) {}
        await new Promise((r) => setTimeout(r, 50));
      } else {
        await new Promise((r) => setTimeout(r, 350));
      }
    }
  }
  if (!isHudMode) {
    obsFrameLoop();
  }

  cameraDeviceSelect.addEventListener("change", () => {
    startWebcam(cameraDeviceSelect.value);
  });

  btnAuto.addEventListener("click", () => {
    manualPointOverride = null;
    const nextAuto = !(serverState && serverState.auto_mode);
    postJson("/api/select", nextAuto ? { target_id: "__auto__" } : { auto_mode: false, target_id: serverState?.active_target_id });
  });

  btnNext.addEventListener("click", () => {
    manualPointOverride = null;
    postJson("/api/select", { target_id: "__next__" });
  });

  cvModeSelect.addEventListener("change", () => {
    postJson("/api/settings", { cv_mode: cvModeSelect.value });
  });

  btnSimShips.addEventListener("click", () => {
    const next = !(serverState && serverState.enable_sim_vessels);
    postJson("/api/settings", { enable_sim_vessels: next });
  });

  btnRings.addEventListener("click", () => {
    showZoomRings = !showZoomRings;
    btnRings.classList.toggle("active", showZoomRings);
  });

  btnLandmarks.addEventListener("click", () => {
    showLandmarks = !showLandmarks;
    btnLandmarks.classList.toggle("active", showLandmarks);
  });

  document.querySelectorAll(".zoom-btn").forEach((btn) => {
    btn.addEventListener("click", () => {
      document.querySelectorAll(".zoom-btn").forEach((b) => b.classList.remove("active"));
      btn.classList.add("active");
      const z = parseFloat(btn.dataset.z);
      if (manualPointOverride && z > 0) manualPointOverride.zoom_x = z;
      postJson("/api/settings", { zoom_x: z > 0 ? z : null });
    });
  });

  btnSaveCalib.addEventListener("click", () => {
    postJson("/api/settings", {
      calibration: {
        az_deg: parseFloat(document.getElementById("calAz").value),
        tilt_deg: parseFloat(document.getElementById("calTilt").value),
        roll_deg: parseFloat(document.getElementById("calRoll").value),
        hfov_deg: parseFloat(document.getElementById("calHfov").value),
        alt_m: parseFloat(document.getElementById("calAlt").value),
        k1: parseFloat(document.getElementById("calK1").value),
        adsb_lead_s: parseFloat(document.getElementById("calLead").value),
      },
    });
  });

  const visualTracks = new Map(); // id -> { u, v, du_s, dv_s, seenNow }

  // Click on synoptic view: either lock onto nearest target OR point PTZ directly at clicked coordinate!
  synCanvas.addEventListener("click", (ev) => {
    const rect = synCanvas.getBoundingClientRect();
    const u = (ev.clientX - rect.left) / rect.width;
    const v = (ev.clientY - rect.top) / rect.height;

    let bestTarget = null;
    let bestDist = 0.038;
    for (const t of serverState?.targets || []) {
      const vt = visualTracks.get(t.id);
      const tu = vt ? vt.u : t.u;
      const tv = vt ? vt.v : t.v;
      const d = Math.hypot(tu - u, tv - v);
      if (d < bestDist) {
        bestDist = d;
        bestTarget = t;
      }
    }

    if (bestTarget) {
      manualPointOverride = null;
      postJson("/api/select", { target_id: bestTarget.id, auto_mode: false });
    } else {
      manualPointOverride = {
        u,
        v,
        zoom_x: cam.zoom > 3 ? cam.zoom : 15.0,
        label: v < 0.56 ? "MANUAL SKY POINT" : "MANUAL RIVER POINT",
      };
      postJson("/api/settings", { auto_mode: false });
    }
  });

  // --- Lightweight Synoptic Computer Vision Spotter (Runs on Live Camera Water & Sky Regions) ---
  const cvCanvas = document.createElement("canvas");
  cvCanvas.width = 320;
  cvCanvas.height = 180;
  const cvCtx = cvCanvas.getContext("2d", { willReadFrequently: true });
  let bgFloat = null;
  let trackedBlobs = [];
  let nextBlobId = 1;

  function inRiverOrSkyMask(u, v) {
    // Exclude foreground piers, Little Island floodlight reflections (v > 0.72),
    // and West Side Highway car headlight reflections (u < 0.46, v > 0.63)
    if (v > 0.72) return null;
    if (u < 0.46 && v > 0.63) return null;
    if (u < 0.35 && v > 0.50) return null;
    // Open sky corridor above Manhattan/JC skylines
    if (v >= 0.08 && v < 0.47) return "sky";
    // Mid-channel Hudson River away from pier light shimmer
    if (u >= 0.37 && v >= 0.58 && v <= 0.72) return "water";
    return null;
  }

  function runSynopticCVSpotter() {
    const cvDot = document.getElementById("cvDot");
    const cvStatus = document.getElementById("cvStatus");
    if (serverState?.cv_mode === "off") {
      cvDot.className = "dot";
      cvStatus.textContent = "CV Spotter: OFF";
      return;
    }
    const liveSource =
      videoMode === "obs" && obsBitmap
        ? obsBitmap
        : videoMode === "webcam" && webcamVideo.readyState >= 2
        ? webcamVideo
        : null;
    if (!liveSource) {
      cvDot.className = "dot";
      cvStatus.textContent = "CV Spotter: standby (waiting for Live Cam)";
      return;
    }

    cvCtx.drawImage(liveSource, 0, 0, 320, 180);
    const frame = cvCtx.getImageData(0, 0, 320, 180).data;
    const nPix = 320 * 180;

    if (!bgFloat) {
      bgFloat = new Float32Array(nPix);
      for (let i = 0; i < nPix; i++) {
        bgFloat[i] = 0.299 * frame[i * 4] + 0.587 * frame[i * 4 + 1] + 0.114 * frame[i * 4 + 2];
      }
      return;
    }

    // Detect moving foreground pixels in masked sky/river zones (higher threshold rejects wave/glass shimmer)
    const gridW = 40, gridH = 22;
    const cellCounts = new Uint16Array(gridW * gridH);
    const cellSumU = new Float32Array(gridW * gridH);
    const cellSumV = new Float32Array(gridW * gridH);

    for (let y = 0; y < 180; y++) {
      const v = y / 180;
      for (let x = 0; x < 320; x++) {
        const u = x / 320;
        const idx = y * 320 + x;
        const lum = 0.299 * frame[idx * 4] + 0.587 * frame[idx * 4 + 1] + 0.114 * frame[idx * 4 + 2];
        const diff = Math.abs(lum - bgFloat[idx]);
        bgFloat[idx] = bgFloat[idx] * 0.92 + lum * 0.08;

        if (diff > 28.0 && inRiverOrSkyMask(u, v)) {
          const gx = Math.min(gridW - 1, Math.floor(u * gridW));
          const gy = Math.min(gridH - 1, Math.floor(v * gridH));
          const gi = gy * gridW + gx;
          cellCounts[gi]++;
          cellSumU[gi] += u;
          cellSumV[gi] += v;
        }
      }
    }

    const rawCandidates = [];
    for (let gy = 0; gy < gridH; gy++) {
      for (let gx = 0; gx < gridW; gx++) {
        const gi = gy * gridW + gx;
        if (cellCounts[gi] >= 5 && cellCounts[gi] <= 45) {
          const cu = cellSumU[gi] / cellCounts[gi];
          const cv = cellSumV[gi] / cellCounts[gi];
          const region = inRiverOrSkyMask(cu, cv);
          if (region) rawCandidates.push({ u: cu, v: cv, region });
        }
      }
    }

    // Associate with trackedBlobs & enforce linear trajectory coherence (rejects shimmering wave reflections!)
    const now = performance.now() / 1000;
    for (const cand of rawCandidates) {
      let best = null;
      let bestD = 0.025;
      for (const b of trackedBlobs) {
        const d = Math.hypot(b.u - cand.u, b.v - cand.v);
        if (d < bestD) {
          bestD = d;
          best = b;
        }
      }
      if (best) {
        const stepDist = Math.hypot(cand.u - best.u, cand.v - best.v);
        best.totalPath += stepDist;
        best.u = 0.65 * best.u + 0.35 * cand.u;
        best.v = 0.65 * best.v + 0.35 * cand.v;
        const elapsed = Math.max(0.25, now - best.startTs);
        best.du_s = (best.u - best.startU) / elapsed;
        best.dv_s = (best.v - best.startV) / elapsed;
        best.hits++;
        best.ts = now;
      } else {
        trackedBlobs.push({
          id: nextBlobId++,
          startU: cand.u,
          startV: cand.v,
          startTs: now,
          totalPath: 0.0,
          u: cand.u,
          v: cand.v,
          du_s: 0,
          dv_s: 0,
          hits: 1,
          region: cand.region,
          ts: now,
        });
      }
    }

    trackedBlobs = trackedBlobs.filter((b) => now - b.ts < 0.9);
    const confirmed = trackedBlobs
      .filter((b) => {
        const netDisp = Math.hypot(b.u - b.startU, b.v - b.startV);
        const efficiency = netDisp / Math.max(0.0001, b.totalPath);
        const speed = Math.hypot(b.du_s, b.dv_s);
        // Must persist >= 3.3s, travel >= 3.2% across frame in a straight line (efficiency >= 0.72), at realistic speed
        return b.hits >= 15 && netDisp >= 0.032 && efficiency >= 0.72 && speed >= 0.003 && speed <= 0.06;
      })
      .slice(0, 4);

    cvDot.className = "dot live";
    cvStatus.textContent = `CV (${serverState?.cv_mode || "assist"}): ${confirmed.length} coherent`;
    postJson("/api/cv_detections", { detections: confirmed });
  }

  if (!isHudMode) {
    setInterval(runSynopticCVSpotter, 220);
  }

  // --- Poll Backend World State ---
  let calibInitialized = false;
  async function pollState() {
    try {
      const res = await fetch("/api/state");
      serverState = await res.json();
      lastPollTs = performance.now();
      updateDashboardUI(serverState);
    } catch (_) {}
  }
  setInterval(pollState, 350);
  pollState();
  if (autoWebcam) {
    videoSourceSelect.value = "webcam";
    startWebcam();
  }

  function updateDashboardUI(st) {
    if (!st || isObsMode || isHudMode) return;

    const adsbDot = document.getElementById("adsbDot");
    const adsbStatus = document.getElementById("adsbStatus");
    const airCount = st.targets.filter((t) => t.kind !== "vessel").length;
    adsbDot.className = st.adsb_source !== "offline" ? "dot live" : "dot";
    adsbStatus.textContent = `ADS-B (${st.adsb_source}): ${airCount} in view`;

    const aisDot = document.getElementById("aisDot");
    const aisStatus = document.getElementById("aisStatus");
    const shipCount = st.targets.filter((t) => t.kind === "vessel").length;
    const isLiveAis = st.ais_source === "local_ais" || st.ais_source === "aisstream_live";
    aisDot.className = isLiveAis ? "dot live" : "dot sim";
    const aisLabel =
      st.ais_source === "local_ais"
        ? `local SDR: ${shipCount} in view / ${st.ais_local_count || 0} RF`
        : st.ais_source === "aisstream_live"
        ? `aisstream live: ${shipCount} in view / ${st.ais_harbor_total || 0} harbor`
        : `sim: ${shipCount} boats`;
    aisStatus.textContent = `AIS (${aisLabel})`;

    const obsCamDot = document.getElementById("obsCamDot");
    const obsCamStatus = document.getElementById("obsCamStatus");
    if (st.obs_cam) {
      obsCamDot.className = st.obs_cam.connected ? "dot live" : "dot";
      obsCamStatus.textContent = st.obs_cam.connected
        ? `OBS Cam: LIVE (${obsBitmap ? obsBitmap.width + "x" + obsBitmap.height : "standby"})`
        : `OBS Cam: ${st.obs_cam.status_text || "disconnected"}`;
      obsCamStatus.title = st.obs_cam.status_text || "";
      if (st.obs_cam.url && document.activeElement !== obsCamInput && !obsCamInput.value) {
        obsCamInput.value = st.obs_cam.url;
      }
    }

    btnAuto.classList.toggle("active", st.auto_mode && !manualPointOverride);
    btnAuto.textContent = `Auto-Director: ${st.auto_mode && !manualPointOverride ? "ON" : "MANUAL"}`;

    btnSimShips.classList.toggle("active", st.enable_sim_vessels);
    btnSimShips.textContent = `Sim Boats: ${st.enable_sim_vessels ? "ON" : "OFF"}`;
    if (st.cv_mode && document.activeElement !== cvModeSelect) {
      cvModeSelect.value = st.cv_mode;
    }

    document.getElementById("dwellLabel").textContent =
      st.auto_mode && st.active_target_id ? `Next switch in: ${st.dwell_remaining_s.toFixed(0)}s` : "Manual lock";

    if (!calibInitialized && st.calibration) {
      calibInitialized = true;
      document.getElementById("calAz").value = st.calibration.az_deg;
      document.getElementById("calTilt").value = st.calibration.tilt_deg;
      document.getElementById("calRoll").value = st.calibration.roll_deg;
      document.getElementById("calHfov").value = st.calibration.hfov_deg;
      document.getElementById("calAlt").value = st.calibration.alt_m;
      if (st.calibration.k1 !== undefined) {
        document.getElementById("calK1").value = st.calibration.k1;
      }
      if (st.calibration.adsb_lead_s !== undefined) {
        document.getElementById("calLead").value = st.calibration.adsb_lead_s;
      }
    }

    // Target table
    const tbody = document.getElementById("targetTableBody");
    tbody.innerHTML = "";
    for (const t of st.targets) {
      const tr = document.createElement("tr");
      if (t.id === st.active_target_id && !manualPointOverride) tr.className = "selected";
      const altOrLen = t.kind === "vessel" ? `${Math.round(t.length_m)}m hull` : `${t.alt_ft.toLocaleString()} ft`;
      tr.innerHTML = `
        <td><span class="tag ${t.kind}">${t.kind}${t.cv_locked ? " +CV" : ""}</span></td>
        <td><strong>${t.callsign}</strong></td>
        <td style="color:var(--muted)">${t.subtitle}</td>
        <td>${altOrLen}</td>
        <td>${t.gs_kts > 0 ? t.gs_kts + " kt" : "—"}</td>
        <td>${t.dist_km.toFixed(2)} km</td>
        <td>${t.rec_zoom_x.toFixed(0)}x</td>
        <td><strong>${t.interest.toFixed(0)}</strong></td>
      `;
      tr.addEventListener("click", () => {
        manualPointOverride = null;
        postJson("/api/select", { target_id: t.id, auto_mode: false });
      });
      tbody.appendChild(tr);
    }
  }

  // Draw crisp true-angular-scale target silhouette in PTZ viewport so you can preview
  // exact optical PTZ object resolution even when zooming a 1080p/4K webcam image!
  function drawOpticalScaleSilhouette(ctx, W, H, target, zoomX, hfovDeg) {
    if (!target || silhouetteSelect.value === "raw") return;
    const ptzHfovDeg = hfovDeg / zoomX;
    const objDeg = (2 * Math.atan2(target.length_m * 0.5, target.dist_km * 1000) * 180) / Math.PI;
    const pxWidth = Math.max(18, (objDeg / ptzHfovDeg) * W);
    const cx = W * 0.5;
    const cy = H * 0.5;

    ctx.save();
    ctx.translate(cx, cy);

    // Subtle targeting brackets + true optical scale silhouette
    const bracketW = Math.min(W * 0.85, Math.max(70, pxWidth * 1.35));
    const bracketH = Math.max(52, bracketW * 0.45);
    const cLen = 14;
    ctx.strokeStyle =
      target.kind === "helicopter" ? "#f59e0b" : target.kind === "vessel" ? "#10b981" : "#38bdf8";
    ctx.lineWidth = 2;

    const corners = [
      [-bracketW / 2, -bracketH / 2, 1, 1],
      [bracketW / 2, -bracketH / 2, -1, 1],
      [-bracketW / 2, bracketH / 2, 1, -1],
      [bracketW / 2, bracketH / 2, -1, -1],
    ];
    for (const [bx, by, sx, sy] of corners) {
      ctx.beginPath();
      ctx.moveTo(bx + sx * cLen, by);
      ctx.lineTo(bx, by);
      ctx.lineTo(bx, by + sy * cLen);
      ctx.stroke();
    }

    // Scale ruler bar showing exact 4K Optical PTZ pixel width
    const optical4kPx = Math.round((objDeg / ptzHfovDeg) * 3840);
    const barY = bracketH / 2 + 18;
    ctx.strokeStyle = "rgba(255,255,255,0.75)";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(-pxWidth / 2, barY);
    ctx.lineTo(pxWidth / 2, barY);
    ctx.moveTo(-pxWidth / 2, barY - 5);
    ctx.lineTo(-pxWidth / 2, barY + 5);
    ctx.moveTo(pxWidth / 2, barY - 5);
    ctx.lineTo(pxWidth / 2, barY + 5);
    ctx.stroke();

    ctx.font = "600 13px -apple-system, BlinkMacSystemFont, monospace";
    ctx.fillStyle = "rgba(255,255,255,0.92)";
    ctx.textAlign = "center";
    ctx.fillText(
      `True ${target.length_m}m size @ ${zoomX.toFixed(1)}x = ${optical4kPx} px wide on 4K Optical PTZ`,
      0,
      barY + 18
    );

    ctx.restore();
  }

  // --- Main 60fps Render, C1 Visual Track Smoother & Velocity Feed-Forward Loop ---
  let lastFrameTs = performance.now();

  function render(now) {
    const dt = Math.min(0.06, (now - lastFrameTs) / 1000);
    lastFrameTs = now;

    let srcMedia = snapshotImg;
    let mediaReady = snapshotImg.complete && snapshotImg.naturalWidth > 0;
    let srcW = snapshotImg.naturalWidth;
    let srcH = snapshotImg.naturalHeight;

    if (videoMode === "obs" && obsBitmap) {
      srcMedia = obsBitmap;
      mediaReady = true;
      srcW = obsBitmap.width;
      srcH = obsBitmap.height;
    } else if (videoMode === "webcam" && webcamVideo.readyState >= 2 && webcamVideo.videoWidth > 0) {
      srcMedia = webcamVideo;
      mediaReady = true;
      srcW = webcamVideo.videoWidth;
      srcH = webcamVideo.videoHeight;
    }

    const hfov = serverState?.calibration?.hfov_deg || 67.957;
    const dtSincePoll = Math.min(3.5, (now - lastPollTs) / 1000);

    // Advance 60fps C1 visual tracks so poll arrivals blend smoothly with zero pixel jumps
    const blendPos = 1.0 - Math.exp(-dt / 0.55);
    const blendVel = 1.0 - Math.exp(-dt / 0.45);
    for (const t of serverState?.targets || []) {
      const tDu = t.du_s !== undefined ? t.du_s : (t.pan_dps || 0) / hfov;
      const tDv = t.dv_s !== undefined ? t.dv_s : -(t.tilt_dps || 0) / (hfov * 0.5625);
      const srvU = t.u + tDu * dtSincePoll;
      const srvV = t.v + tDv * dtSincePoll;
      const vt = visualTracks.get(t.id);
      if (!vt || Math.hypot(srvU - vt.u, srvV - vt.v) > 0.08) {
        visualTracks.set(t.id, { u: srvU, v: srvV, du_s: tDu, dv_s: tDv, seenNow: now });
      } else {
        vt.u += vt.du_s * dt;
        vt.v += vt.dv_s * dt;
        vt.u += (srvU - vt.u) * blendPos;
        vt.v += (srvV - vt.v) * blendPos;
        vt.du_s += (tDu - vt.du_s) * blendVel;
        vt.dv_s += (tDv - vt.dv_s) * blendVel;
        vt.seenNow = now;
      }
    }
    for (const [id, vt] of visualTracks.entries()) {
      if (now - vt.seenNow > 2500) visualTracks.delete(id);
    }

    // Determine active target with continuous velocity feed-forward
    let activeTarget = null;
    if (manualPointOverride) {
      activeTarget = {
        callsign: manualPointOverride.label,
        subtitle: "Manual Synoptic Click Override",
        kind: "aircraft",
        u: manualPointOverride.u,
        v: manualPointOverride.v,
        du_s: 0,
        dv_s: 0,
        rec_zoom_x: manualPointOverride.zoom_x,
        dist_km: 1.8,
        length_m: 25,
        gs_kts: 0,
        alt_ft: 0,
      };
    } else if (serverState && serverState.active_target_id) {
      const base = serverState.targets.find((t) => t.id === serverState.active_target_id);
      if (base) {
        const vt = visualTracks.get(base.id);
        activeTarget = vt
          ? { ...base, u: vt.u, v: vt.v, du_s: vt.du_s, dv_s: vt.dv_s }
          : base;
      }
    }

    const desiredU = activeTarget ? Math.max(0.03, Math.min(0.97, activeTarget.u)) : 0.5;
    const desiredV = activeTarget ? Math.max(0.03, Math.min(0.97, activeTarget.v)) : 0.5;
    const desiredZoom = manualPointOverride
      ? manualPointOverride.zoom_x
      : serverState?.ptz?.zoom_x || (activeTarget ? activeTarget.rec_zoom_x : 1.0);

    // Critically damped spring-damper + exact image-space velocity feed-forward
    const omega = 4.6;
    const ffU = activeTarget ? activeTarget.du_s || 0 : 0;
    const ffV = activeTarget ? activeTarget.dv_s || 0 : 0;

    const au = omega * omega * (desiredU - cam.u) + 2 * omega * (ffU - cam.vu);
    const av = omega * omega * (desiredV - cam.v) + 2 * omega * (ffV - cam.vv);
    cam.vu += au * dt;
    cam.vv += av * dt;
    cam.u = Math.max(0.01, Math.min(0.99, cam.u + cam.vu * dt));
    cam.v = Math.max(0.01, Math.min(0.99, cam.v + cam.vv * dt));

    const az = 4.0 * 4.0 * (desiredZoom - cam.zoom) - 2 * 4.0 * cam.vzoom;
    cam.vzoom += az * dt;
    cam.zoom = Math.max(1.0, Math.min(40.0, cam.zoom + cam.vzoom * dt));

    // 1) Draw Synoptic Canvas (or transparent AR overlay in ?hud=1 mode)
    if (!isObsMode && (mediaReady || isHudMode)) {
      const W = synCanvas.width;
      const H = synCanvas.height;
      if (isHudMode) {
        synCtx.clearRect(0, 0, W, H);
      } else {
        synCtx.drawImage(srcMedia, 0, 0, W, H);
      }

      // Optional 3D Skyline Calibration Pins
      if (showLandmarks && serverState?.landmarks) {
        for (const lm of serverState.landmarks) {
          const lx = lm.u * W;
          const ly = lm.v * H;
          synCtx.fillStyle = "#fde047";
          synCtx.beginPath();
          synCtx.arc(lx, ly, 4.5, 0, Math.PI * 2);
          synCtx.fill();
          synCtx.font = "600 16px -apple-system, sans-serif";
          synCtx.fillStyle = "rgba(0,0,0,0.75)";
          synCtx.fillRect(lx + 7, ly - 12, synCtx.measureText(lm.name).width + 10, 22);
          synCtx.fillStyle = "#fde047";
          synCtx.fillText(lm.name, lx + 12, ly + 4);
        }
      }

      // Draw all live targets using silky C1 smoothed coordinates & exact velocity vectors
      for (const t of serverState?.targets || []) {
        const vt = visualTracks.get(t.id);
        const u = vt ? vt.u : t.u;
        const v = vt ? vt.v : t.v;
        const duS = vt ? vt.du_s : (t.du_s || 0);
        const dvS = vt ? vt.dv_s : (t.dv_s || 0);
        const x = u * W;
        const y = v * H;
        const isSel = serverState?.active_target_id === t.id && !manualPointOverride;
        const col =
          t.source.startsWith("cv_")
            ? "#e879f9"
            : t.kind === "helicopter"
            ? "#f59e0b"
            : t.kind === "vessel"
            ? "#10b981"
            : "#38bdf8";

        // Velocity lead vector (8 sec projection, capped at 160px so ultra-close helicopters don't span the screen)
        const vxPx = duS * 8.0 * W;
        const vyPx = dvS * 8.0 * H;
        const vLen = Math.hypot(vxPx, vyPx);
        if (vLen > 4.0) {
          const vScale = vLen > 160.0 ? 160.0 / vLen : 1.0;
          const lx = x + vxPx * vScale;
          const ly = y + vyPx * vScale;
          synCtx.strokeStyle = col;
          synCtx.lineWidth = 2;
          synCtx.beginPath();
          synCtx.moveTo(x, y);
          synCtx.lineTo(lx, ly);
          synCtx.stroke();
        }

        synCtx.strokeStyle = col;
        synCtx.lineWidth = isSel ? 3.5 : 2;
        synCtx.beginPath();
        synCtx.arc(x, y, isSel ? 16 : 10, 0, Math.PI * 2);
        synCtx.stroke();

        // Callsign pill
        synCtx.font = "600 17px -apple-system, sans-serif";
        const lbl = `${t.callsign} (${t.dist_km.toFixed(1)}km)`;
        const tw = synCtx.measureText(lbl).width;
        synCtx.fillStyle = "rgba(9, 13, 22, 0.78)";
        synCtx.fillRect(x + 14, y - 13, tw + 12, 24);
        synCtx.fillStyle = col;
        synCtx.fillText(lbl, x + 20, y + 4);
      }

      // Optional nested 5x / 10x / 20x / 30x zoom reference boxes
      if (showZoomRings) {
        const rings = [
          [5, "#94a3b8"],
          [10, "#38bdf8"],
          [20, "#f59e0b"],
          [30, "#ef4444"],
        ];
        for (const [zx, col] of rings) {
          const rw = W / zx;
          const rh = H / zx;
          const rx = Math.max(0, Math.min(W - rw, cam.u * W - rw * 0.5));
          const ry = Math.max(0, Math.min(H - rh, cam.v * H - rh * 0.5));
          synCtx.strokeStyle = col;
          synCtx.setLineDash([6, 4]);
          synCtx.lineWidth = 1.8;
          synCtx.strokeRect(rx, ry, rw, rh);
          synCtx.setLineDash([]);
          synCtx.font = "700 15px monospace";
          synCtx.fillStyle = col;
          synCtx.fillText(`${zx}x`, rx + 5, Math.max(16, ry - 4));
        }
      }

      // Active PTZ FOV Reticle
      if (!isHudMode || showHudPtzBox) {
        const boxW = W / cam.zoom;
        const boxH = H / cam.zoom;
        const boxX = Math.max(0, Math.min(W - boxW, cam.u * W - boxW * 0.5));
        const boxY = Math.max(0, Math.min(H - boxH, cam.v * H - boxH * 0.5));

        synCtx.strokeStyle = "#ffffff";
        synCtx.lineWidth = 2.5;
        synCtx.strokeRect(boxX, boxY, boxW, boxH);
        synCtx.font = "700 17px ui-monospace, monospace";
        synCtx.fillStyle = "#ffffff";
        synCtx.fillText(`PTZ ${cam.zoom.toFixed(1)}x`, boxX + 6, Math.max(20, boxY - 6));
      }
    }

    // 2) Draw PTZ Zoomed Output Canvas
    if (!isHudMode && mediaReady) {
      const W = ptzCanvas.width;
      const H = ptzCanvas.height;
      const cropW = srcW / cam.zoom;
      const cropH = srcH / cam.zoom;
      const cropX = Math.max(0, Math.min(srcW - cropW, cam.u * srcW - cropW * 0.5));
      const cropY = Math.max(0, Math.min(srcH - cropH, cam.v * srcH - cropH * 0.5));

      ptzCtx.imageSmoothingEnabled = true;
      ptzCtx.imageSmoothingQuality = "high";
      ptzCtx.drawImage(srcMedia, cropX, cropY, cropW, cropH, 0, 0, W, H);

      if (!isObsMode) {
        drawOpticalScaleSilhouette(ptzCtx, W, H, activeTarget, cam.zoom, hfov);
      }
    }

    // Update PTZ HUD & Broadcast Lower-Third Chyron
    const panDeg = (cam.u - 0.5) * hfov;
    const tiltDeg = (0.5 - cam.v) * hfov * 0.5625;
    document.getElementById("ptzHudTop").textContent =
      `PAN ${panDeg >= 0 ? "+" : ""}${panDeg.toFixed(1)}°   TILT ${tiltDeg >= 0 ? "+" : ""}${tiltDeg.toFixed(1)}°   ZOOM ${cam.zoom.toFixed(1)}x`;

    const chyron = document.getElementById("chyron");
    const cTitle = document.getElementById("chyronTitle");
    const cBadge = document.getElementById("chyronBadge");
    const cSub = document.getElementById("chyronSub");

    if (activeTarget) {
      chyron.className =
        "chyron " +
        (activeTarget.source?.startsWith("cv_")
          ? "cv"
          : activeTarget.kind === "helicopter"
          ? "heli"
          : activeTarget.kind === "vessel"
          ? "vessel"
          : "");
      cTitle.textContent = activeTarget.callsign;
      cBadge.textContent = activeTarget.kind.toUpperCase();
      const parts = [activeTarget.subtitle];
      if (activeTarget.kind !== "vessel" && activeTarget.alt_ft > 0) {
        parts.push(`${activeTarget.alt_ft.toLocaleString()} ft`);
      } else if (activeTarget.length_m) {
        parts.push(`${Math.round(activeTarget.length_m)}m length`);
      }
      if (activeTarget.gs_kts > 0) parts.push(`${activeTarget.gs_kts} kts`);
      parts.push(`${activeTarget.dist_km.toFixed(2)} km away`);
      cSub.textContent = parts.join("  •  ");
    } else {
      chyron.className = "chyron";
      cTitle.textContent = "HUDSON RIVER LIVE CORRIDOR";
      cBadge.textContent = "WIDE";
      cSub.textContent = "Chelsea • Little Island • Lower New York Harbor";
    }

    requestAnimationFrame(render);
  }

  requestAnimationFrame(render);
})();
