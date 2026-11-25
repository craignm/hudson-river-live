(() => {
  const urlParams = new URLSearchParams(window.location.search);
  let ytVideoId = urlParams.get("v") || null;
  let ytPlayer = null;
  let ytReady = false;
  let ytApiLoaded = false;

  let state = null;
  let libraryItems = [];
  let currentLibFilter = "all";
  let currentPhotoItem = null;

  let enhanceTriggeredHere = false;
  let lastEnhanceRunSeen = 0;

  let slewPreviewUntil = 0;
  let optToggleA = true;
  let optInFlight = false;
  let toastTimer = null;

  // DOM Elements
  const streamBadge = document.getElementById("streamBadge");
  const streamBadgeText = document.getElementById("streamBadgeText");
  const ptzTelemetryText = document.getElementById("ptzTelemetryText");
  const btnAutoDirector = document.getElementById("btnAutoDirector");
  const autoDirectorLabel = document.getElementById("autoDirectorLabel");
  const btnNextTarget = document.getElementById("btnNextTarget");
  const btnEnhance = document.getElementById("btnEnhance");
  const btnLibrary = document.getElementById("btnLibrary");
  const libraryBtnLabel = document.getElementById("libraryBtnLabel");
  const btnFinderToggle = document.getElementById("btnFinderToggle");
  const btnFinderClose = document.getElementById("btnFinderClose");
  const ytZoomLink = document.getElementById("ytZoomLink");
  const btnFullscreen = document.getElementById("btnFullscreen");

  const enhanceBanner = document.getElementById("enhanceBanner");
  const enhanceBannerText = document.getElementById("enhanceBannerText");
  const enhanceProgressFill = document.getElementById("enhanceProgressFill");
  const enhancePctText = document.getElementById("enhancePctText");

  const ytZoomWrap = document.getElementById("ytZoomWrap");
  const opticalLayer = document.getElementById("opticalLayer");
  const optImgA = document.getElementById("optImgA");
  const optImgB = document.getElementById("optImgB");
  const clickSurface = document.getElementById("clickSurface");
  const clickRipple = document.getElementById("clickRipple");
  const slewPill = document.getElementById("slewPill");
  const slewPillText = document.getElementById("slewPillText");
  const toastPill = document.getElementById("toastPill");

  const finderPip = document.getElementById("finderPip");
  const finderWrap = document.getElementById("finderWrap");
  const finderBgImg = document.getElementById("finderBgImg");
  const finderCanvas = document.getElementById("finderCanvas");

  const chyronCard = document.getElementById("chyronCard");
  const chyronBadge = document.getElementById("chyronBadge");
  const chyronTitle = document.getElementById("chyronTitle");
  const chyronSub = document.getElementById("chyronSub");

  const liveTargetsRow = document.getElementById("liveTargetsRow");
  const presetsRow = document.getElementById("presetsRow");
  const celestialRow = document.getElementById("celestialRow");

  const libraryModal = document.getElementById("libraryModal");
  const libraryGrid = document.getElementById("libraryGrid");
  const btnCloseLibrary = document.getElementById("btnCloseLibrary");

  const compareModal = document.getElementById("compareModal");
  const cmpTitle = document.getElementById("cmpTitle");
  const cmpStats = document.getElementById("cmpStats");
  const compareCanvas = document.getElementById("compareCanvas");
  const cmpLoading = document.getElementById("cmpLoading");
  const wipeSlider = document.getElementById("wipeSlider");
  const cmpDownloadLink = document.getElementById("cmpDownloadLink");
  const btnSharePhoto = document.getElementById("btnSharePhoto");
  const btnBackToLibrary = document.getElementById("btnBackToLibrary");
  const btnCloseCompare = document.getElementById("btnCloseCompare");

  function showToast(msg, ms = 2600) {
    if (!toastPill) return;
    toastPill.textContent = msg;
    toastPill.classList.remove("hidden");
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => toastPill.classList.add("hidden"), ms);
  }

  function triggerSlewPreview(label) {
    slewPreviewUntil = Date.now() + 3000;
    if (slewPill && slewPillText) {
      slewPillText.textContent = label ? `Slewing to ${label}…` : "Slewing optical zoom…";
      slewPill.classList.remove("hidden");
    }
    syncVideoLayers();
    if (!ytVideoId) {
      pullOpticalFrame(true);
    }
  }

  let ytPausedSince = 0;

  function snapYouTubeToLiveEdge(forceSeek = false) {
    if (!ytPlayer || !ytReady || document.hidden) return;
    try {
      const pState = typeof ytPlayer.getPlayerState === "function" ? ytPlayer.getPlayerState() : 1;
      const dur = typeof ytPlayer.getDuration === "function" ? ytPlayer.getDuration() : 0;
      const cur = typeof ytPlayer.getCurrentTime === "function" ? ytPlayer.getCurrentTime() : 0;
      const lag = dur > 0 && cur > 0 ? dur - cur : 0;

      // Auto-resume & snap to live if paused (immediately on user PTZ action, or after 12s idle pause)
      if (pState === 2 || pState === -1 || pState === 5) {
        if (!ytPausedSince) ytPausedSince = Date.now();
        if (forceSeek || Date.now() - ytPausedSince > 12000) {
          ytPausedSince = 0;
          ytPlayer.playVideo?.();
          if (dur > 0) ytPlayer.seekTo?.(dur + 2, true);
        }
        return;
      }
      ytPausedSince = 0;

      if (dur > 0 && (forceSeek ? lag > 3.5 : lag > 11.0)) {
        ytPlayer.seekTo?.(dur + 2, true);
        ytPlayer.setPlaybackRate?.(1.0);
        streamBadge.classList.remove("behind");
        if (ytVideoId) streamBadgeText.textContent = "LIVE 1080p60";
      } else if (dur > 0 && lag > 5.5) {
        // Silent 1.25× catch-up glides back to live edge without a seek spinner
        ytPlayer.setPlaybackRate?.(1.25);
        streamBadge.classList.add("behind");
        if (ytVideoId) streamBadgeText.textContent = `SYNCING LIVE (-${Math.round(lag)}s)`;
      } else {
        if (typeof ytPlayer.getPlaybackRate === "function" && ytPlayer.getPlaybackRate() !== 1) {
          ytPlayer.setPlaybackRate?.(1.0);
        }
        streamBadge.classList.remove("behind");
        if (ytVideoId) streamBadgeText.textContent = "LIVE 1080p60";
      }
    } catch (_) {}
  }

  streamBadge?.addEventListener("click", () => snapYouTubeToLiveEdge(true));
  document.addEventListener("visibilitychange", () => {
    if (!document.hidden) {
      setTimeout(() => snapYouTubeToLiveEdge(true), 250);
    }
  });
  setInterval(() => snapYouTubeToLiveEdge(false), 3000);

  async function postSelect(payload, slewLabel) {
    snapYouTubeToLiveEdge(true);
    triggerSlewPreview(slewLabel);
    try {
      const r = await fetch("/api/zoom/select", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
      });
      const data = await r.json();
      if (!r.ok || data.ok === false) {
        showToast(data.error || "Unable to move camera right now");
      }
      pollState();
    } catch (_) {
      showToast("Connection error — retrying…");
    }
  }

  // --- 1. YouTube 1080p60 Stream + Optical Layer Management ---
  function ensureYouTubePlayer(videoId) {
    ytVideoId = videoId;
    ytZoomWrap.classList.remove("hidden");
    ytZoomLink.href = `https://www.youtube.com/watch?v=${videoId}`;
    ytZoomLink.classList.remove("hidden");
    streamBadge.classList.remove("optical");
    streamBadgeText.textContent = "LIVE 1080p60";
    syncVideoLayers();

    if (ytPlayer && ytReady && typeof ytPlayer.loadVideoById === "function") {
      ytPlayer.loadVideoById(videoId);
      return;
    }

    window.onYouTubeIframeAPIReady = () => {
      ytPlayer = new YT.Player("ytZoomPlayer", {
        videoId: ytVideoId,
        playerVars: {
          autoplay: 1,
          mute: 1,
          controls: 1,
          rel: 0,
          playsinline: 1,
          modestbranding: 1,
          iv_load_policy: 3,
        },
        events: {
          onReady: (ev) => {
            ytReady = true;
            ev.target.mute();
            ev.target.playVideo();
            syncVideoLayers();
          },
          onStateChange: (ev) => {
            if (ev.data === YT.PlayerState.PLAYING && ytPlayer.setPlaybackQualityRange) {
              try {
                ytPlayer.setPlaybackQualityRange("hd1080", "hd1080");
              } catch (_) {}
            }
          },
        },
      });
    };

    if (!ytApiLoaded) {
      ytApiLoaded = true;
      const s = document.createElement("script");
      s.src = "https://www.youtube.com/iframe_api";
      document.head.appendChild(s);
    } else if (window.YT && window.YT.Player) {
      window.onYouTubeIframeAPIReady();
    }
  }

  function syncVideoLayers() {
    const now = Date.now();
    const inSlew = now < slewPreviewUntil;
    if (!inSlew && slewPill) {
      slewPill.classList.add("hidden");
    }
    if (ytVideoId) {
      // Keep the smooth YouTube video visible 100% of the time (never cover with JPEG snapshots)
      opticalLayer.classList.add("transparent-over-yt");
      opticalLayer.classList.remove("slew-preview-active");
    } else {
      streamBadge.classList.add("optical");
      streamBadgeText.textContent = "LIVE OPTICAL";
      opticalLayer.classList.remove("transparent-over-yt");
      opticalLayer.classList.remove("slew-preview-active");
    }
  }

  async function checkZoomYouTubeStream() {
    if (urlParams.has("v")) {
      ensureYouTubePlayer(urlParams.get("v"));
      return;
    }
    try {
      const r = await fetch("/api/zoom/live", { cache: "no-store" });
      if (!r.ok) return;
      const data = await r.json();
      if (data.isLive && data.videoId) {
        if (data.videoId !== ytVideoId) {
          ensureYouTubePlayer(data.videoId);
        }
      }
    } catch (_) {}
  }

  function pullOpticalFrame(force = false) {
    if (document.hidden || ytVideoId) return;
    if (!force && optInFlight) return;
    if (optInFlight) return;

    optInFlight = true;
    const nextImg = optToggleA ? optImgB : optImgA;
    const curImg = optToggleA ? optImgA : optImgB;
    const loader = new Image();
    loader.onload = () => {
      nextImg.src = loader.src;
      nextImg.classList.add("visible");
      curImg.classList.remove("visible");
      optToggleA = !optToggleA;
      optInFlight = false;
    };
    loader.onerror = () => {
      optInFlight = false;
    };
    loader.src = `/api/zoom/frame.jpg?t=${Date.now()}`;
  }

  // --- 2. Click-to-Center on Zoomed Video, Zoom Pills & Nudge Pad ---
  clickSurface.addEventListener("click", (ev) => {
    const rect = clickSurface.getBoundingClientRect();
    const fullH = document.getElementById("videoFrame").getBoundingClientRect().height;
    const rx = Math.max(0, Math.min(1, (ev.clientX - rect.left) / rect.width));
    const ry = Math.max(0, Math.min(1, (ev.clientY - rect.top) / fullH));

    clickRipple.style.left = `${ev.clientX - rect.left}px`;
    clickRipple.style.top = `${ev.clientY - rect.top}px`;
    clickRipple.classList.remove("hidden");
    clickRipple.style.animation = "none";
    void clickRipple.offsetWidth;
    clickRipple.style.animation = "";

    postSelect({ ptz_click: { rx, ry } }, "clicked point");
  });

  document.querySelectorAll(".z-btn").forEach((btn) => {
    btn.addEventListener("click", () => {
      const z = parseFloat(btn.dataset.z || "0");
      document.querySelectorAll(".z-btn").forEach((b) => b.classList.toggle("active", b === btn));
      postSelect({ zoom_x: z > 0 ? z : null }, z > 0 ? `${z}× zoom` : "auto zoom");
    });
  });

  document.querySelectorAll(".nudge-btn").forEach((btn) => {
    btn.addEventListener("click", () => {
      const sx = parseFloat(btn.dataset.sx || "0");
      const sy = parseFloat(btn.dataset.sy || "0");
      postSelect({ ptz_nudge: { step_x: sx, step_y: sy } }, "fine pan/tilt");
    });
  });

  btnAutoDirector.addEventListener("click", () => {
    document.querySelectorAll(".z-btn").forEach((b) => b.classList.toggle("active", b.dataset.z === "0"));
    postSelect({ target_id: "__auto__" }, "Auto-Director");
  });

  btnNextTarget.addEventListener("click", () => {
    postSelect({ target_id: "__next__" }, "next target");
  });

  // --- 3. Wide Harbor Finder PiP ---
  let finderOpen = true;
  function setFinderVisible(vis) {
    finderOpen = vis;
    finderPip.classList.toggle("hidden", !vis);
    btnFinderToggle.setAttribute("aria-pressed", String(vis));
    if (vis) refreshFinderBackdrop();
  }
  btnFinderToggle.addEventListener("click", () => setFinderVisible(!finderOpen));
  btnFinderClose.addEventListener("click", () => setFinderVisible(false));

  function refreshFinderBackdrop() {
    if (!finderOpen || document.hidden) return;
    finderBgImg.src = `/api/zoom/wide.jpg?t=${Math.floor(Date.now() / 3000)}`;
  }

  finderWrap.addEventListener("click", (ev) => {
    const rect = finderWrap.getBoundingClientRect();
    const u = Math.max(0, Math.min(1, (ev.clientX - rect.left) / rect.width));
    const v = Math.max(0, Math.min(1, (ev.clientY - rect.top) / rect.height));
    const curZ = state?.ptz?.zoom_x || 15;
    postSelect(
      {
        manual_point: {
          u,
          v,
          zoom_x: curZ < 4 ? 15 : curZ,
          label: v < 0.54 ? "HARBOR SKYLINE POINT" : "HUDSON RIVER CHANNEL",
          subtitle: "Selected via Wide Harbor Finder",
        },
      },
      "harbor point"
    );
  });

  function drawFinderOverlay() {
    if (!finderOpen || !state || !finderCanvas) return;
    const ctx = finderCanvas.getContext("2d");
    const W = finderCanvas.width;
    const H = finderCanvas.height;
    ctx.clearRect(0, 0, W, H);

    for (const t of state.targets || []) {
      const tx = t.u * W;
      const ty = t.v * H;
      ctx.fillStyle = t.kind === "vessel" ? "#34d399" : "#fbbf24";
      ctx.beginPath();
      ctx.arc(tx, ty, t.id === state.active_target_id ? 7 : 4.5, 0, Math.PI * 2);
      ctx.fill();
    }

    const zoom = Math.max(1, state.ptz?.zoom_x || 1);
    let cu = 0.5;
    let cv = 0.5;
    if (!state.auto_mode && state.manual_point) {
      cu = state.manual_point.u ?? 0.5;
      cv = state.manual_point.v ?? 0.5;
    } else if (state.active_target_id) {
      const act =
        (state.targets || []).find((x) => x.id === state.active_target_id) ||
        state.active_target;
      if (act) {
        cu = act.u;
        cv = act.v;
      }
    } else if (state.ptz) {
      cu = Math.max(0, Math.min(1, 0.5 + (state.ptz.pan_deg || 0) / 81.2));
      cv = Math.max(0, Math.min(1, 0.5 - (state.ptz.tilt_deg || 0) / 45.7));
    }

    const bw = Math.max(16, W / zoom);
    const bh = Math.max(11, H / zoom);
    const bx = Math.max(2, Math.min(W - bw - 2, cu * W - bw * 0.5));
    const by = Math.max(2, Math.min(H - bh - 2, cv * H - bh * 0.5));
    ctx.strokeStyle = "#38bdf8";
    ctx.lineWidth = 2.6;
    ctx.strokeRect(bx, by, bw, bh);
  }

  // --- 4. Render Telemetry, Chyron & Pointing Chips ---
  let presetsRenderedSig = "";

  function kindIcon(kind) {
    if (kind === "vessel") return "🚢";
    if (kind === "helicopter") return "🚁";
    if (kind === "aircraft") return "✈️";
    if (kind === "celestial") return "🪐";
    return "📍";
  }

  function renderState() {
    if (!state) return;
    const ptz = state.ptz || { pan_deg: 0, tilt_deg: 0, zoom_x: 1 };
    const mp = state.manual_point;

    if (mp && mp.kind === "celestial" && mp.az_deg !== undefined) {
      ptzTelemetryText.textContent = `AZ ${mp.az_deg.toFixed(1)}° · ALT ${mp.alt_deg >= 0 ? "+" : ""}${mp.alt_deg.toFixed(1)}° · ${ptz.zoom_x.toFixed(1)}× [SIDEREAL]`;
    } else {
      const pSign = ptz.pan_deg >= 0 ? "+" : "";
      const tSign = ptz.tilt_deg >= 0 ? "+" : "";
      ptzTelemetryText.textContent = `PAN ${pSign}${ptz.pan_deg.toFixed(1)}° · TILT ${tSign}${ptz.tilt_deg.toFixed(1)}° · ${ptz.zoom_x.toFixed(1)}×`;
    }

    // Auto-Director button state + lease timer
    if (state.auto_mode) {
      btnAutoDirector.classList.add("active");
      btnAutoDirector.classList.remove("manual");
      autoDirectorLabel.textContent = "Auto-Director";
    } else {
      btnAutoDirector.classList.remove("active");
      btnAutoDirector.classList.add("manual");
      const lease = Math.ceil(state.lease_remaining_s || 0);
      autoDirectorLabel.textContent = lease > 0 ? `Manual (${lease}s → Auto)` : "Resume Auto";
    }

    // Library count
    if (typeof state.library_count === "number") {
      libraryBtnLabel.textContent = state.library_count > 0 ? `Library (${state.library_count})` : "Library";
    }

    // Enhance live progress
    const ls = state.lucky_stack;
    if (ls) {
      if (ls.running) {
        btnEnhance.disabled = true;
        btnEnhance.textContent = `⏳ ${ls.progress_pct || 0}%`;
        enhanceBanner.classList.remove("hidden");
        enhanceBannerText.textContent = ls.message || "Enhancing 4K burst…";
        enhanceProgressFill.style.width = `${Math.max(4, ls.progress_pct || 5)}%`;
        enhancePctText.textContent = `${ls.progress_pct || 0}%`;
      } else {
        btnEnhance.disabled = false;
        btnEnhance.textContent = "✨ Enhance!";
        enhanceBanner.classList.add("hidden");
        if (ls.last_run_ts && ls.last_ok) {
          if (lastEnhanceRunSeen === 0 && !enhanceTriggeredHere) {
            lastEnhanceRunSeen = ls.last_run_ts;
          } else if (ls.last_run_ts > lastEnhanceRunSeen) {
            lastEnhanceRunSeen = ls.last_run_ts;
            fetchLibrary().then(() => {
              if (enhanceTriggeredHere && ls.latest_entry) {
                enhanceTriggeredHere = false;
                openCompareModal(ls.latest_entry, false);
              }
            });
          }
        }
      }
    }

    // Active Target & Broadcast Chyron
    let activeObj = null;
    if (!state.auto_mode && mp) {
      activeObj = {
        kind: mp.kind || "landmark",
        title: mp.label || "CUSTOM POINT",
        subtitle: mp.subtitle || "",
        dist_km: mp.dist_km,
        gs_kts: 0,
        alt_ft: 0,
      };
    } else if (state.active_target_id) {
      const t =
        (state.targets || []).find((x) => x.id === state.active_target_id) ||
        state.active_target;
      if (t) {
        activeObj = {
          kind: t.kind,
          title: t.callsign,
          subtitle: t.subtitle,
          dist_km: t.dist_km,
          gs_kts: t.gs_kts,
          alt_ft: t.alt_ft,
          length_m: t.length_m,
        };
      }
    }

    if (activeObj) {
      chyronCard.className = `chyron ${activeObj.kind === "helicopter" ? "heli" : activeObj.kind}`;
      chyronBadge.textContent = activeObj.kind.toUpperCase();
      chyronTitle.textContent = activeObj.title;
      const parts = [];
      if (activeObj.subtitle) parts.push(activeObj.subtitle);
      if ((activeObj.kind === "aircraft" || activeObj.kind === "helicopter") && activeObj.alt_ft > 0) {
        parts.push(`${activeObj.alt_ft.toLocaleString()} ft`);
      } else if (activeObj.kind === "vessel" && activeObj.length_m) {
        parts.push(`${Math.round(activeObj.length_m)}m`);
      }
      if (activeObj.gs_kts > 0) parts.push(`${activeObj.gs_kts} kts`);
      if (activeObj.dist_km && activeObj.kind !== "celestial") {
        parts.push(`${Number(activeObj.dist_km).toFixed(2)} km`);
      }
      parts.push(`${ptz.zoom_x.toFixed(1)}×`);
      chyronSub.textContent = parts.join("  ·  ");
    } else {
      chyronCard.className = "chyron";
      chyronBadge.textContent = "WIDE";
      chyronTitle.textContent = "HUDSON RIVER LIVE CORRIDOR";
      chyronSub.textContent = "Chelsea · Little Island · Lower New York Harbor";
    }

    // Row 1: Live AIS Ships & ADS-B Aircraft
    const targets = state.targets || [];
    liveTargetsRow.innerHTML = "";
    if (targets.length === 0) {
      const span = document.createElement("span");
      span.className = "dock-empty";
      span.textContent = "Quiet harbor moment — select a landmark or sky object below";
      liveTargetsRow.appendChild(span);
    } else {
      for (const t of targets.slice(0, 14)) {
        const btn = document.createElement("button");
        const isAct = state.active_target_id === t.id && (state.auto_mode || !mp);
        btn.type = "button";
        btn.className = `target-pill ${t.kind}${isAct ? " active" : ""}`;
        const meta =
          t.kind === "vessel"
            ? `${t.gs_kts > 0.4 ? t.gs_kts.toFixed(1) + " kn · " : ""}${t.dist_km.toFixed(1)} km`
            : `${t.alt_ft > 0 ? Math.round(t.alt_ft / 100) * 100 + " ft · " : ""}${t.dist_km.toFixed(1)} km`;
        btn.innerHTML = `<span>${kindIcon(t.kind)} ${t.callsign}</span><span class="pill-meta">${meta}</span>`;
        btn.addEventListener("click", () => {
          document.querySelectorAll(".z-btn").forEach((b) => b.classList.toggle("active", b.dataset.z === "0"));
          postSelect({ target_id: t.id, auto_mode: false }, t.callsign);
        });
        liveTargetsRow.appendChild(btn);
      }
    }

    // Row 2a: Landmarks
    const presets = state.presets || [];
    const activePid = !state.auto_mode
      ? mp?.preset_id
      : state.active_target?.preset_id || null;
    const pSig = presets.map((p) => p.id).join(",") + "|" + (activePid || "");
    if (pSig !== presetsRenderedSig) {
      presetsRenderedSig = pSig;
      presetsRow.innerHTML = "";
      for (const pr of presets) {
        const btn = document.createElement("button");
        btn.type = "button";
        btn.className = `target-pill landmark${activePid === pr.id ? " active" : ""}`;
        btn.title = `${pr.subtitle} (${pr.dist_km} km · ${pr.zoom_x}×)`;
        btn.innerHTML = `<span>${pr.name}</span><span class="pill-meta">${pr.dist_km}km</span>`;
        btn.addEventListener("click", () => {
          if (!state.auto_mode && activePid === pr.id) {
            postSelect({ target_id: "__auto__" }, "Auto-Director");
          } else {
            document.querySelectorAll(".z-btn").forEach((b) => b.classList.toggle("active", b.dataset.z === "0"));
            postSelect({ preset_id: pr.id }, pr.name);
          }
        });
        presetsRow.appendChild(btn);
      }
    }

    // Row 2b: Celestial Ephemeris
    const celList = state.celestial || [];
    const activeCid = !state.auto_mode ? mp?.celestial_id : null;
    celestialRow.innerHTML = "";
    const celIcons = { moon: "🌙", saturn: "🪐", jupiter: "🪐", venus: "✨" };
    for (const c of celList) {
      const btn = document.createElement("button");
      btn.type = "button";
      const vis = !!c.visible;
      btn.className = `target-pill celestial${activeCid === c.id ? " active" : ""}${!vis ? " unavailable" : ""}`;
      btn.title = vis
        ? `${c.label} — Alt +${c.alt_deg.toFixed(1)}°, Az ${c.az_deg.toFixed(1)}° (Click to track)`
        : `${c.name}: ${c.status_note}`;
      const altTxt = vis ? `+${Math.round(c.alt_deg)}°` : "set";
      btn.innerHTML = `<span>${celIcons[c.id] || "✨"} ${c.name}</span><span class="pill-meta">${altTxt}</span>`;
      btn.addEventListener("click", () => {
        if (!vis) {
          showToast(`${c.name}: ${c.status_note}`);
          return;
        }
        if (activeCid === c.id) {
          postSelect({ target_id: "__auto__" }, "Auto-Director");
        } else {
          document.querySelectorAll(".z-btn").forEach((b) => b.classList.toggle("active", b.dataset.z === "0"));
          postSelect({ celestial_id: c.id }, c.name);
        }
      });
      celestialRow.appendChild(btn);
    }

    drawFinderOverlay();
  }

  async function pollState() {
    if (document.hidden) return;
    try {
      const r = await fetch("/api/zoom/state", { cache: "no-store" });
      if (!r.ok) return;
      state = await r.json();
      renderState();
      syncVideoLayers();
    } catch (_) {}
  }

  // --- 5. Enhance! & Persistent Library Gallery + 4K Split-Wipe Viewer ---
  btnEnhance.addEventListener("click", async () => {
    enhanceTriggeredHere = true;
    btnEnhance.disabled = true;
    btnEnhance.textContent = "⏳ Starting…";
    try {
      const r = await fetch("/api/zoom/enhance", { method: "POST", body: "{}" });
      const data = await r.json();
      if (!r.ok || data.ok === false) {
        enhanceTriggeredHere = false;
        btnEnhance.disabled = false;
        btnEnhance.textContent = "✨ Enhance!";
        showToast(data.error || "Enhance busy — try in a moment");
        return;
      }
      pollState();
    } catch (_) {
      enhanceTriggeredHere = false;
      btnEnhance.disabled = false;
      btnEnhance.textContent = "✨ Enhance!";
      showToast("Unable to start Enhance right now");
    }
  });

  async function fetchLibrary() {
    try {
      const r = await fetch("/api/zoom/library", { cache: "no-store" });
      if (!r.ok) return [];
      const data = await r.json();
      libraryItems = data.items || [];
      renderLibraryGrid();
      return libraryItems;
    } catch (_) {
      return [];
    }
  }

  function renderLibraryGrid() {
    if (!libraryGrid) return;
    libraryGrid.innerHTML = "";
    const filtered = libraryItems.filter((it) => {
      if (currentLibFilter === "all") return true;
      if (currentLibFilter === "vessel") {
        return it.kind === "vessel" || it.kind === "aircraft" || it.kind === "helicopter";
      }
      return it.kind === currentLibFilter;
    });

    if (filtered.length === 0) {
      libraryGrid.innerHTML = `<div class="dock-empty" style="padding:24px;">No captures in this category yet — aim at a target and click <strong>✨ Enhance!</strong></div>`;
      return;
    }

    for (const item of filtered) {
      const card = document.createElement("button");
      card.type = "button";
      card.className = "lib-card";
      const dtTxt = item.timestamp
        ? new Date(item.timestamp * 1000).toLocaleString([], {
            month: "short",
            day: "numeric",
            hour: "numeric",
            minute: "2-digit",
          })
        : "";
      card.innerHTML = `
        <div class="lib-thumb-wrap">
          <img src="/api/zoom/library/${item.thumb_file}" alt="${item.title}" loading="lazy" />
          <span class="lib-snr-badge">${item.snr_improvement_x}× SNR</span>
        </div>
        <div class="lib-card-body">
          <div class="lib-card-title">${item.title}</div>
          <div class="lib-card-sub">${item.subtitle || `${item.width}×${item.height} · ${item.zoom_x}× Optical`}</div>
          <div class="lib-card-time">${dtTxt} · ${item.frames_stacked}/${item.frames_captured} frames</div>
        </div>
      `;
      card.addEventListener("click", () => openCompareModal(item, true));
      libraryGrid.appendChild(card);
    }
  }

  document.querySelectorAll("#libFilterTabs .ftab").forEach((tab) => {
    tab.addEventListener("click", () => {
      currentLibFilter = tab.dataset.filter || "all";
      document.querySelectorAll("#libFilterTabs .ftab").forEach((t) => t.classList.toggle("active", t === tab));
      renderLibraryGrid();
    });
  });

  btnLibrary.addEventListener("click", () => {
    libraryModal.classList.remove("hidden");
    fetchLibrary();
  });

  btnCloseLibrary.addEventListener("click", () => {
    libraryModal.classList.add("hidden");
  });

  // 4K Interactive Split-Wipe Canvas State
  let cmpBeforeImg = null;
  let cmpAfterImg = null;
  let cmpSplit = 0.5;
  let cmpZoom = 1.0;
  let cmpPanU = 0.5;
  let cmpPanV = 0.5;
  let cmpDragMode = null;
  let cmpDragStart = { x: 0, y: 0, u: 0.5, v: 0.5 };

  function drawCompareCanvas() {
    if (!compareCanvas || !cmpBeforeImg || !cmpAfterImg) return;
    const ctx = compareCanvas.getContext("2d");
    const W = compareCanvas.width;
    const H = compareCanvas.height;
    const srcW = cmpAfterImg.naturalWidth || 3840;
    const srcH = cmpAfterImg.naturalHeight || 2160;

    const cropW = srcW / cmpZoom;
    const cropH = srcH / cmpZoom;
    const halfU = 0.5 / cmpZoom;
    const halfV = 0.5 / cmpZoom;
    cmpPanU = Math.max(halfU, Math.min(1 - halfU, cmpPanU));
    cmpPanV = Math.max(halfV, Math.min(1 - halfV, cmpPanV));
    const sx = cmpPanU * srcW - cropW * 0.5;
    const sy = cmpPanV * srcH - cropH * 0.5;

    ctx.clearRect(0, 0, W, H);
    ctx.imageSmoothingEnabled = cmpZoom < 2.2;
    ctx.imageSmoothingQuality = "high";

    const splitX = Math.round(cmpSplit * W);
    ctx.drawImage(cmpAfterImg, sx, sy, cropW, cropH, 0, 0, W, H);
    if (splitX > 0) {
      ctx.save();
      ctx.beginPath();
      ctx.rect(0, 0, splitX, H);
      ctx.clip();
      ctx.drawImage(cmpBeforeImg, sx, sy, cropW, cropH, 0, 0, W, H);
      ctx.restore();
    }

    ctx.strokeStyle = "#38bdf8";
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(splitX, 0);
    ctx.lineTo(splitX, H);
    ctx.stroke();

    ctx.fillStyle = "rgba(9, 14, 24, 0.9)";
    ctx.beginPath();
    ctx.arc(splitX, H * 0.5, 22, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = "#f1f5f9";
    ctx.font = "700 15px system-ui, sans-serif";
    ctx.textAlign = "center";
    ctx.fillText("◀▶", splitX, H * 0.5 + 5);

    ctx.font = "700 16px -apple-system, BlinkMacSystemFont, sans-serif";
    if (splitX > 170) {
      ctx.fillStyle = "rgba(9, 14, 24, 0.8)";
      ctx.fillRect(16, 16, 228, 32);
      ctx.fillStyle = "#fca5a5";
      ctx.textAlign = "left";
      ctx.fillText("BEFORE (Single Turbulent Frame)", 26, 37);
    }
    if (W - splitX > 200) {
      ctx.fillStyle = "rgba(9, 14, 24, 0.8)";
      ctx.fillRect(W - 318, 16, 302, 32);
      ctx.fillStyle = "#6ee7b7";
      ctx.textAlign = "right";
      ctx.fillText("ENHANCED (4K Lucky Stack + RL)", W - 26, 37);
    }
  }

  function openCompareModal(item, cameFromLibrary) {
    if (!item) return;
    currentPhotoItem = item;
    libraryModal.classList.add("hidden");
    compareModal.classList.remove("hidden");
    btnBackToLibrary.classList.toggle("hidden", !cameFromLibrary && libraryItems.length <= 1);
    window.history.replaceState(null, "", `#photo=${encodeURIComponent(item.id)}`);

    cmpZoom = 1.0;
    cmpPanU = 0.5;
    cmpPanV = 0.5;
    cmpSplit = 0.5;
    wipeSlider.value = "500";
    document.querySelectorAll(".cmp-zbtn").forEach((b) => b.classList.toggle("active", b.dataset.cz === "1"));

    cmpTitle.textContent = `✨ ${item.title}`;
    const when = item.timestamp ? new Date(item.timestamp * 1000).toLocaleString() : "";
    cmpStats.textContent = `${item.width || 3840}×${item.height || 2160} @ ${(item.zoom_x || 30).toFixed(0)}×  ·  ${item.frames_stacked}/${item.frames_captured} frames  ·  ${item.snr_improvement_x}× SNR${when ? "  ·  " + when : ""}`;

    const dlSlug = (item.title || "hudson_enhanced").toLowerCase().replace(/[^a-z0-9]+/g, "_");
    cmpDownloadLink.href = `/api/zoom/library/${item.after_png}`;
    cmpDownloadLink.download = `${dlSlug}_4k.png`;

    cmpLoading.classList.remove("hidden");
    const bImg = new Image();
    const aImg = new Image();
    let done = 0;
    const finish = () => {
      done += 1;
      if (done === 2) {
        cmpBeforeImg = bImg;
        cmpAfterImg = aImg;
        cmpLoading.classList.add("hidden");
        drawCompareCanvas();
      }
    };
    bImg.onload = finish;
    aImg.onload = finish;
    bImg.src = `/api/zoom/library/${item.before_jpg}`;
    aImg.src = `/api/zoom/library/${item.after_jpg}`;
  }

  function closeCompareModal() {
    compareModal.classList.add("hidden");
    if (window.location.hash.startsWith("#photo=")) {
      window.history.replaceState(null, "", window.location.pathname + window.location.search);
    }
  }

  btnCloseCompare.addEventListener("click", closeCompareModal);
  btnBackToLibrary.addEventListener("click", () => {
    closeCompareModal();
    libraryModal.classList.remove("hidden");
    fetchLibrary();
  });

  btnSharePhoto.addEventListener("click", async () => {
    if (!currentPhotoItem) return;
    const shareUrl = `${window.location.origin}/zoom#photo=${encodeURIComponent(currentPhotoItem.id)}`;
    try {
      await navigator.clipboard.writeText(shareUrl);
      showToast("Copied link to Enhanced capture!");
    } catch (_) {
      showToast(shareUrl);
    }
  });

  wipeSlider.addEventListener("input", () => {
    cmpSplit = parseInt(wipeSlider.value, 10) / 1000;
    drawCompareCanvas();
  });

  document.querySelectorAll(".cmp-zbtn").forEach((btn) => {
    btn.addEventListener("click", () => {
      cmpZoom = parseFloat(btn.dataset.cz || "1");
      document.querySelectorAll(".cmp-zbtn").forEach((b) => b.classList.toggle("active", b === btn));
      drawCompareCanvas();
    });
  });

  compareCanvas.addEventListener("pointerdown", (ev) => {
    const rect = compareCanvas.getBoundingClientRect();
    const rx = (ev.clientX - rect.left) / rect.width;
    if (Math.abs(rx - cmpSplit) < 0.045 || cmpZoom <= 1.01) {
      cmpDragMode = "wipe";
      cmpSplit = Math.max(0.02, Math.min(0.98, rx));
      wipeSlider.value = String(Math.round(cmpSplit * 1000));
      drawCompareCanvas();
    } else {
      cmpDragMode = "pan";
      cmpDragStart = { x: ev.clientX, y: ev.clientY, u: cmpPanU, v: cmpPanV };
    }
    compareCanvas.setPointerCapture(ev.pointerId);
  });

  compareCanvas.addEventListener("pointermove", (ev) => {
    if (!cmpDragMode) return;
    const rect = compareCanvas.getBoundingClientRect();
    if (cmpDragMode === "wipe") {
      cmpSplit = Math.max(0.02, Math.min(0.98, (ev.clientX - rect.left) / rect.width));
      wipeSlider.value = String(Math.round(cmpSplit * 1000));
      drawCompareCanvas();
    } else if (cmpDragMode === "pan") {
      const dx = (ev.clientX - cmpDragStart.x) / rect.width;
      const dy = (ev.clientY - cmpDragStart.y) / rect.height;
      cmpPanU = cmpDragStart.u - dx / cmpZoom;
      cmpPanV = cmpDragStart.v - dy / cmpZoom;
      drawCompareCanvas();
    }
  });

  compareCanvas.addEventListener("pointerup", () => {
    cmpDragMode = null;
  });

  compareCanvas.addEventListener(
    "wheel",
    (ev) => {
      ev.preventDefault();
      const factor = ev.deltaY < 0 ? 1.18 : 1 / 1.18;
      cmpZoom = Math.max(1.0, Math.min(6.0, cmpZoom * factor));
      document.querySelectorAll(".cmp-zbtn").forEach((b) => b.classList.remove("active"));
      drawCompareCanvas();
    },
    { passive: false }
  );

  // Fullscreen & Keyboard Shortcuts
  function toggleFullscreen() {
    if (!document.fullscreenElement) {
      document.documentElement.requestFullscreen?.().catch(() => {});
    } else {
      document.exitFullscreen?.().catch(() => {});
    }
  }
  btnFullscreen.addEventListener("click", toggleFullscreen);

  window.addEventListener("keydown", (ev) => {
    if (ev.target && /INPUT|TEXTAREA/.test(ev.target.tagName)) return;
    if (ev.key === "Escape") {
      if (!compareModal.classList.contains("hidden")) closeCompareModal();
      else if (!libraryModal.classList.contains("hidden")) libraryModal.classList.add("hidden");
      return;
    }
    if (!compareModal.classList.contains("hidden") || !libraryModal.classList.contains("hidden")) return;

    if (ev.key === "f" || ev.key === "F") toggleFullscreen();
    else if (ev.key === "w" || ev.key === "W") setFinderVisible(!finderOpen);
    else if (ev.key === "ArrowLeft") postSelect({ ptz_nudge: { step_x: -0.32, step_y: 0 } }, "pan left");
    else if (ev.key === "ArrowRight") postSelect({ ptz_nudge: { step_x: 0.32, step_y: 0 } }, "pan right");
    else if (ev.key === "ArrowUp") postSelect({ ptz_nudge: { step_x: 0, step_y: 0.32 } }, "tilt up");
    else if (ev.key === "ArrowDown") postSelect({ ptz_nudge: { step_x: 0, step_y: -0.32 } }, "tilt down");
  });

  // --- 6. Boot Loops & Deep-Link Check ---
  document.addEventListener("visibilitychange", () => {
    if (!document.hidden) {
      snapYouTubeToLiveEdge();
      pollState();
    }
  });

  checkZoomYouTubeStream();
  pollState();
  pullOpticalFrame(true);
  refreshFinderBackdrop();

  setInterval(pollState, 1000);
  setInterval(() => pullOpticalFrame(false), 850);
  setInterval(refreshFinderBackdrop, 3500);
  setInterval(checkZoomYouTubeStream, 45000);

  if (window.location.hash.startsWith("#photo=")) {
    const targetId = decodeURIComponent(window.location.hash.slice("#photo=".length));
    fetchLibrary().then((items) => {
      const found = items.find((x) => x.id === targetId);
      if (found) openCompareModal(found, false);
    });
  }
})();
