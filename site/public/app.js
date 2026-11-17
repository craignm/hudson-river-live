(() => {
  const DEFAULT_VIDEO_ID = "kE_N1WY7UAY";
  const CHANNEL_ID = "UCTtqzzGVBuwkACkLPau0GTQ";
  const CAMERA_LAT = 40.7448;
  const CAMERA_LON = -74.0081;
  const NOAA_STATION = "8518750"; // The Battery, NY

  const urlParams = new URLSearchParams(window.location.search);
  let currentVideoId = urlParams.get("v") || DEFAULT_VIDEO_ID;
  let player = null;
  let playerReady = false;
  let idleTimer = null;

  // DOM elements
  const nycClockEl = document.getElementById("nycClock");
  const weatherValEl = document.getElementById("weatherVal");
  const tideValEl = document.getElementById("tideVal");
  const tideValMobileEl = document.getElementById("tideValMobile");
  const liveBadge = document.getElementById("liveBadge");
  const liveBadgeText = document.getElementById("liveBadgeText");
  const ytWatchLink = document.getElementById("ytWatchLink");

  const btnLandmarks = document.getElementById("btnLandmarks");
  const landmarksLayer = document.getElementById("landmarksLayer");
  const btnFitFill = document.getElementById("btnFitFill");
  const fitFillLabel = document.getElementById("fitFillLabel");
  const btnFullscreen = document.getElementById("btnFullscreen");
  const btnInfo = document.getElementById("btnInfo");
  const btnCloseInfo = document.getElementById("btnCloseInfo");
  const infoDrawer = document.getElementById("infoDrawer");
  const fileProtocolNotice = document.getElementById("fileProtocolNotice");

  // --- 1. Local file:// guard (YouTube blocks embeds without http/https Referer) ---
  if (window.location.protocol === "file:") {
    fileProtocolNotice.classList.remove("hidden");
  }

  // --- 2. Auto-hiding HUD on mouse stillness ---
  function wakeHud() {
    document.body.classList.remove("user-idle");
    clearTimeout(idleTimer);
    idleTimer = setTimeout(() => {
      if (!document.body.classList.contains("drawer-open")) {
        document.body.classList.add("user-idle");
      }
    }, 3600);
  }

  ["mousemove", "mousedown", "touchstart", "keydown"].forEach((evt) => {
    window.addEventListener(evt, wakeHud, { passive: true });
  });
  wakeHud();

  // --- 3. YouTube IFrame Player Setup ---
  function updateWatchLinks(videoId, watchUrl) {
    const href = watchUrl || `https://www.youtube.com/watch?v=${videoId}`;
    ytWatchLink.href = href;
  }

  updateWatchLinks(currentVideoId);

  window.onYouTubeIframeAPIReady = function () {
    if (window.location.protocol === "file:") return;

    player = new YT.Player("ytPlayer", {
      videoId: currentVideoId,
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
        onReady: (event) => {
          playerReady = true;
          event.target.mute();
          event.target.playVideo();
        },
        onStateChange: (event) => {
          // Suggest crisp HD/4K quality once buffered
          if (event.data === YT.PlayerState.PLAYING && player.setPlaybackQualityRange) {
            try {
              player.setPlaybackQualityRange("hd1440", "hd2160");
            } catch (_) {}
          }
        },
        onError: () => {
          // Fallback to permanent channel live stream embed if videoId ever expires on static host
          const box = document.getElementById("stageBox");
          const existing = document.getElementById("ytPlayer");
          if (existing && box) {
            const iframe = document.createElement("iframe");
            iframe.src = `https://www.youtube.com/embed/live_stream?channel=${CHANNEL_ID}&autoplay=1&mute=1&rel=0&playsinline=1`;
            iframe.allow = "accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share";
            iframe.allowFullscreen = true;
            iframe.referrerPolicy = "strict-origin-when-cross-origin";
            existing.replaceWith(iframe);
          }
        },
      },
    });
  };

  if (window.location.protocol !== "file:") {
    const tag = document.createElement("script");
    tag.src = "https://www.youtube.com/iframe_api";
    document.head.appendChild(tag);
  }

  // --- 4. Auto-healing Active Stream Lookup (/api/live on Cloudflare Pages) ---
  async function checkActiveStream() {
    if (urlParams.has("v") || window.location.protocol === "file:") return;
    try {
      const resp = await fetch("/api/live", { cache: "no-store" });
      if (!resp.ok) return;
      const contentType = resp.headers.get("content-type") || "";
      if (!contentType.includes("application/json")) return;

      const data = await resp.json();
      if (data.watchUrl) {
        updateWatchLinks(data.videoId || currentVideoId, data.watchUrl);
      }

      if (data.isLive === false) {
        liveBadge.classList.add("standby");
        liveBadgeText.textContent = "REPLAY";
      } else {
        liveBadge.classList.remove("standby");
        liveBadgeText.textContent = "LIVE";
      }

      if (data.videoId && data.videoId !== currentVideoId) {
        currentVideoId = data.videoId;
        if (player && playerReady && typeof player.loadVideoById === "function") {
          player.loadVideoById(currentVideoId);
          player.mute();
        }
      }
    } catch (_) {
      // Running as static site without /api/live function; default videoId remains active
    }
  }

  checkActiveStream();
  setInterval(checkActiveStream, 5 * 60 * 1000);

  // --- 5. UI Controls: Landmarks, Fit/Fill, Fullscreen, Info Drawer ---
  function toggleLandmarks() {
    const isHidden = landmarksLayer.classList.toggle("hidden");
    btnLandmarks.setAttribute("aria-pressed", String(!isHidden));
  }

  function toggleFitFill() {
    const isFill = document.body.classList.toggle("fill-mode");
    btnFitFill.setAttribute("aria-pressed", String(isFill));
    fitFillLabel.textContent = isFill ? "Fit" : "Fill";
  }

  function toggleFullscreen() {
    if (!document.fullscreenElement) {
      document.documentElement.requestFullscreen?.();
    } else {
      document.exitFullscreen?.();
    }
  }

  function setInfoOpen(open) {
    infoDrawer.classList.toggle("hidden", !open);
    document.body.classList.toggle("drawer-open", open);
    btnInfo.setAttribute("aria-expanded", String(open));
    if (open) wakeHud();
  }

  btnLandmarks.addEventListener("click", toggleLandmarks);
  btnFitFill.addEventListener("click", toggleFitFill);
  btnFullscreen.addEventListener("click", toggleFullscreen);
  btnInfo.addEventListener("click", () => setInfoOpen(infoDrawer.classList.contains("hidden")));
  btnCloseInfo.addEventListener("click", () => setInfoOpen(false));

  window.addEventListener("keydown", (e) => {
    if (e.metaKey || e.ctrlKey || e.altKey) return;
    const key = e.key.toLowerCase();
    if (key === "l") toggleLandmarks();
    else if (key === "z") toggleFitFill();
    else if (key === "f") toggleFullscreen();
    else if (key === "i") setInfoOpen(infoDrawer.classList.contains("hidden"));
    else if (key === "escape") setInfoOpen(false);
  });

  // --- 6. Live NYC Clock ---
  const nycFormatter = new Intl.DateTimeFormat("en-US", {
    timeZone: "America/New_York",
    hour: "numeric",
    minute: "2-digit",
    second: "2-digit",
  });

  function tickClock() {
    nycClockEl.textContent = nycFormatter.format(new Date());
  }
  tickClock();
  setInterval(tickClock, 1000);

  // --- 7. Live Chelsea / Hudson Weather (Open-Meteo, no API key needed) ---
  function degToCompass(deg) {
    const dirs = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"];
    return dirs[Math.round(((deg % 360) / 22.5)) % 16];
  }

  async function fetchWeather() {
    try {
      const url = `https://api.open-meteo.com/v1/forecast?latitude=${CAMERA_LAT}&longitude=${CAMERA_LON}&current=temperature_2m,wind_speed_10m,wind_direction_10m&temperature_unit=fahrenheit&wind_speed_unit=kn`;
      const resp = await fetch(url);
      if (!resp.ok) return;
      const data = await resp.json();
      const cur = data.current;
      if (!cur) return;
      const temp = Math.round(cur.temperature_2m);
      const windKt = Math.round(cur.wind_speed_10m);
      const dir = degToCompass(cur.wind_direction_10m || 0);
      weatherValEl.textContent = `${temp}°F · ${dir} ${windKt} kt`;
    } catch (_) {
      weatherValEl.textContent = "—";
    }
  }

  fetchWeather();
  setInterval(fetchWeather, 10 * 60 * 1000);

  // --- 8. Live Hudson River Tide (NOAA CO-OPS Station 8518750, The Battery) ---
  const shortTimeFmt = new Intl.DateTimeFormat("en-US", {
    timeZone: "America/New_York",
    hour: "numeric",
    minute: "2-digit",
  });

  async function fetchTide() {
    try {
      const now = new Date();
      const yyyy = now.getUTCFullYear();
      const mm = String(now.getUTCMonth() + 1).padStart(2, "0");
      const dd = String(now.getUTCDate()).padStart(2, "0");
      const beginDate = `${yyyy}${mm}${dd}`;

      const [levelResp, predResp] = await Promise.all([
        fetch(
          `https://api.tidesandcurrents.noaa.gov/api/prod/datagetter?date=latest&station=${NOAA_STATION}&product=water_level&datum=MLLW&time_zone=gmt&units=english&format=json`
        ),
        fetch(
          `https://api.tidesandcurrents.noaa.gov/api/prod/datagetter?begin_date=${beginDate}&range=36&station=${NOAA_STATION}&product=predictions&datum=MLLW&time_zone=gmt&interval=hilo&units=english&format=json`
        ),
      ]);

      let levelStr = "";
      if (levelResp.ok) {
        const levelJson = await levelResp.json();
        const latest = levelJson.data && levelJson.data[0];
        if (latest && latest.v) {
          levelStr = `${ parseFloat(latest.v).toFixed(1) } ft`;
        }
      }

      let phaseStr = "";
      if (predResp.ok) {
        const predJson = await predResp.json();
        const preds = predJson.predictions || [];
        const nextEvent = preds.find((p) => {
          const ts = Date.parse(p.t.replace(" ", "T") + ":00Z");
          return ts > now.getTime();
        });
        if (nextEvent) {
          const nextDate = new Date(nextEvent.t.replace(" ", "T") + ":00Z");
          const timeLabel = shortTimeFmt.format(nextDate).toLowerCase().replace(" ", "");
          if (nextEvent.type === "H") {
            phaseStr = `↑ Flood · High ${timeLabel}`;
          } else {
            phaseStr = `↓ Ebb · Low ${timeLabel}`;
          }
        }
      }

      const combined = [levelStr, phaseStr].filter(Boolean).join(" · ") || "—";
      tideValEl.textContent = combined;
      if (tideValMobileEl) tideValMobileEl.textContent = combined;
    } catch (_) {
      tideValEl.textContent = "—";
      if (tideValMobileEl) tideValMobileEl.textContent = "—";
    }
  }

  fetchTide();
  setInterval(fetchTide, 6 * 60 * 1000);
})();
