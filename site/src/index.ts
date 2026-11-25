const CHANNEL_ID = "UCTtqzzGVBuwkACkLPau0GTQ";
const DEFAULT_VIDEO_ID = "kE_N1WY7UAY";
const DEFAULT_ZOOM_VIDEO_ID = "6qHvgTkz5dI";
const TUNNEL_ORIGIN = "https://ptz-tunnel.hudsonriver.live";

interface WorkerEnv {
  PTZ_TUNNEL_SECRET?: string;
  ZOOM_VIDEO_ID?: string;
}

const VALID_LIB_FILE_RE =
  /^[a-z0-9][a-z0-9_-]{2,80}_(thumb\.jpg|after\.jpg|before\.jpg|after\.png)$/;

const ZOOM_TITLE_RE = /\b(zoom|ptz|telephoto|close-?up)\b/i;
const MONTHLY_VOD_RE =
  /\b(at\s+dawn|sunrise|morning|afternoon|sunset|dusk|at\s+night|timelapse|\b20\d{2}\b)\b/i;

interface FeedEntry {
  videoId: string;
  title: string;
}

async function fetchChannelEntries(): Promise<FeedEntry[]> {
  const feedUrl = `https://www.youtube.com/feeds/videos.xml?channel_id=${CHANNEL_ID}`;
  const resp = await fetch(feedUrl, {
    headers: {
      "User-Agent": "hudsonriver.live/1.1",
      Accept: "application/atom+xml,application/xml,text/xml",
    },
    cf: {
      cacheTtl: 60,
      cacheEverything: true,
    },
  });

  if (!resp.ok) {
    throw new Error(`Upstream YouTube feed status ${resp.status}`);
  }

  const entries: FeedEntry[] = [];
  let currentVid = "";
  let currentTitle = "";

  const rewriter = new HTMLRewriter()
    .on("entry", {
      element() {
        currentVid = "";
        currentTitle = "";
      },
    })
    .on("entry > yt\\:videoId, entry > videoId", {
      text(chunk) {
        currentVid += chunk.text;
      },
    })
    .on("entry > title", {
      text(chunk) {
        currentTitle += chunk.text;
        if (chunk.lastInTextNode) {
          const vid = currentVid.trim();
          const title = currentTitle.trim();
          if (vid) {
            entries.push({ videoId: vid, title });
          }
        }
      },
    });

  const transformed = rewriter.transform(resp);
  await transformed.body?.pipeTo(new WritableStream());
  return entries;
}

async function handleLiveLookup(): Promise<Response> {
  try {
    const entries = await fetchChannelEntries();
    // Prefer the canonical 24/7 wide stream ("Hudson River from 17th Street, New York")
    // and strictly exclude PTZ/Zoom streams as well as monthly Dawn/Sunset/Night VOD uploads.
    const canonicalLive = entries.find(
      (e) =>
        /from\s+17th\s+street/i.test(e.title) &&
        !ZOOM_TITLE_RE.test(e.title) &&
        !MONTHLY_VOD_RE.test(e.title)
    );
    const wideMatch = entries.find(
      (e) =>
        /hudson\s*river/i.test(e.title) &&
        !ZOOM_TITLE_RE.test(e.title) &&
        !MONTHLY_VOD_RE.test(e.title)
    );
    const videoId =
      canonicalLive?.videoId || wideMatch?.videoId || DEFAULT_VIDEO_ID;

    return Response.json(
      {
        videoId,
        isLive: true,
        channelId: CHANNEL_ID,
        watchUrl: `https://www.youtube.com/watch?v=${videoId}`,
        checkedAt: new Date().toISOString(),
      },
      {
        headers: {
          "Cache-Control":
            "public, max-age=30, s-maxage=60, stale-while-revalidate=300",
        },
      }
    );
  } catch (err) {
    console.error(
      JSON.stringify({
        event: "youtube_feed_lookup_error",
        error: err instanceof Error ? err.message : String(err),
      })
    );

    return Response.json(
      {
        videoId: DEFAULT_VIDEO_ID,
        isLive: true,
        fallback: true,
        channelId: CHANNEL_ID,
        watchUrl: `https://www.youtube.com/watch?v=${DEFAULT_VIDEO_ID}`,
      },
      {
        headers: {
          "Cache-Control": "public, max-age=30",
        },
      }
    );
  }
}

async function handleZoomLiveLookup(env: WorkerEnv): Promise<Response> {
  const videoId = (env.ZOOM_VIDEO_ID || "").trim() || DEFAULT_ZOOM_VIDEO_ID;
  return Response.json(
    {
      videoId,
      isLive: true,
      channelId: CHANNEL_ID,
      watchUrl: `https://www.youtube.com/watch?v=${videoId}`,
      checkedAt: new Date().toISOString(),
    },
    {
      headers: { "Cache-Control": "public, max-age=20, s-maxage=30" },
    }
  );
}

async function proxyTunnelGet(
  upstreamPath: string,
  env: WorkerEnv,
  edgeTtlSec: number,
  clientCacheCtrl: string
): Promise<Response> {
  const secret = (env.PTZ_TUNNEL_SECRET || "").trim();
  if (!secret) {
    return Response.json(
      { error: "PTZ tunnel not configured" },
      { status: 503 }
    );
  }

  try {
    const upstream = await fetch(`${TUNNEL_ORIGIN}${upstreamPath}`, {
      method: "GET",
      headers: {
        "X-PTZ-Tunnel-Secret": secret,
        Accept: "*/*",
      },
      cf:
        edgeTtlSec > 0
          ? {
              cacheTtl: edgeTtlSec,
              cacheEverything: true,
            }
          : undefined,
    });

    const headers = new Headers();
    const ct = upstream.headers.get("content-type");
    if (ct) headers.set("Content-Type", ct);
    headers.set("Cache-Control", clientCacheCtrl);
    headers.set("X-Content-Type-Options", "nosniff");

    return new Response(upstream.body, {
      status: upstream.status,
      headers,
    });
  } catch (err) {
    return Response.json(
      {
        error: "PTZ observatory temporarily unreachable",
        detail: err instanceof Error ? err.message : String(err),
      },
      { status: 502, headers: { "Cache-Control": "no-store" } }
    );
  }
}

async function proxyTunnelPost(
  request: Request,
  upstreamPath: string,
  env: WorkerEnv
): Promise<Response> {
  const secret = (env.PTZ_TUNNEL_SECRET || "").trim();
  if (!secret) {
    return Response.json(
      { ok: false, error: "PTZ tunnel not configured" },
      { status: 503 }
    );
  }

  const rawBody = await request.text();
  if (rawBody.length > 8192) {
    return Response.json(
      { ok: false, error: "Payload too large" },
      { status: 413 }
    );
  }

  try {
    const upstream = await fetch(`${TUNNEL_ORIGIN}${upstreamPath}`, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
        "X-PTZ-Tunnel-Secret": secret,
      },
      body: rawBody || "{}",
    });

    const text = await upstream.text();
    return new Response(text, {
      status: upstream.status,
      headers: {
        "Content-Type": "application/json",
        "Cache-Control": "no-store",
      },
    });
  } catch (_) {
    return Response.json(
      { ok: false, error: "PTZ observatory temporarily unreachable" },
      { status: 502 }
    );
  }
}

async function handleLibraryAsset(
  request: Request,
  filename: string,
  env: WorkerEnv,
  ctx: ExecutionContext
): Promise<Response> {
  if (!VALID_LIB_FILE_RE.test(filename)) {
    return Response.json({ error: "Invalid asset" }, { status: 400 });
  }

  const cache = caches.default;
  const cacheKey = new Request(
    new URL(`/api/zoom/library/${filename}`, request.url).toString(),
    { method: "GET" }
  );
  const cached = await cache.match(cacheKey);
  if (cached) {
    return cached;
  }

  const resp = await proxyTunnelGet(
    `/api/public/library/${filename}`,
    env,
    86400,
    "public, max-age=31536000, immutable"
  );
  if (resp.status === 200) {
    ctx.waitUntil(cache.put(cacheKey, resp.clone()));
  }
  return resp;
}

export default {
  async fetch(
    request: Request,
    env: WorkerEnv,
    ctx: ExecutionContext
  ): Promise<Response> {
    const url = new URL(request.url);
    const { pathname } = url;

    if (pathname === "/api/live") {
      if (request.method !== "GET" && request.method !== "HEAD") {
        return Response.json({ error: "Method not allowed" }, { status: 405 });
      }
      return handleLiveLookup();
    }

    if (pathname === "/api/zoom/live") {
      if (request.method !== "GET" && request.method !== "HEAD") {
        return Response.json({ error: "Method not allowed" }, { status: 405 });
      }
      return handleZoomLiveLookup(env);
    }

    if (pathname === "/api/zoom/state") {
      if (request.method !== "GET") {
        return Response.json({ error: "Method not allowed" }, { status: 405 });
      }
      return proxyTunnelGet(
        "/api/public/state",
        env,
        1,
        "public, max-age=0, s-maxage=1"
      );
    }

    if (pathname === "/api/zoom/frame.jpg") {
      if (request.method !== "GET") {
        return Response.json({ error: "Method not allowed" }, { status: 405 });
      }
      return proxyTunnelGet(
        "/api/public/frame.jpg",
        env,
        1,
        "no-cache, max-age=0"
      );
    }

    if (pathname === "/api/zoom/wide.jpg") {
      if (request.method !== "GET") {
        return Response.json({ error: "Method not allowed" }, { status: 405 });
      }
      return proxyTunnelGet(
        "/api/public/wide.jpg",
        env,
        2,
        "public, max-age=2, s-maxage=2"
      );
    }

    if (pathname === "/api/zoom/library") {
      if (request.method !== "GET") {
        return Response.json({ error: "Method not allowed" }, { status: 405 });
      }
      return proxyTunnelGet(
        "/api/public/library",
        env,
        2,
        "public, max-age=2, s-maxage=2"
      );
    }

    if (pathname.startsWith("/api/zoom/library/")) {
      if (request.method !== "GET" && request.method !== "HEAD") {
        return Response.json({ error: "Method not allowed" }, { status: 405 });
      }
      const filename = pathname.slice("/api/zoom/library/".length);
      return handleLibraryAsset(request, filename, env, ctx);
    }

    if (pathname === "/api/zoom/select") {
      if (request.method !== "POST") {
        return Response.json({ error: "Method not allowed" }, { status: 405 });
      }
      return proxyTunnelPost(request, "/api/public/select", env);
    }

    if (pathname === "/api/zoom/enhance") {
      if (request.method !== "POST") {
        return Response.json({ error: "Method not allowed" }, { status: 405 });
      }
      return proxyTunnelPost(request, "/api/public/enhance", env);
    }

    if (pathname.startsWith("/api/")) {
      return Response.json({ error: "Not found" }, { status: 404 });
    }

    // Redirect mistyped non-asset paths back to the main live river view
    return Response.redirect(new URL("/", url.origin).toString(), 302);
  },
} satisfies ExportedHandler<WorkerEnv>;
