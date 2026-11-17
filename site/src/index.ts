const CHANNEL_ID = "UCTtqzzGVBuwkACkLPau0GTQ";
const DEFAULT_VIDEO_ID = "kE_N1WY7UAY";

async function handleLiveLookup(): Promise<Response> {
  // Official YouTube Atom feed works reliably from Cloudflare datacenter IPs without consent walls
  const feedUrl = `https://www.youtube.com/feeds/videos.xml?channel_id=${CHANNEL_ID}`;

  try {
    const resp = await fetch(feedUrl, {
      headers: {
        "User-Agent": "hudsonriver.live/1.0",
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

    // Stream the small (~6 KB) Atom XML feed via HTMLRewriter to find the newest Hudson River entry
    let currentEntryVid = "";
    let currentEntryTitle = "";
    let matchedVideoId = "";
    let firstVideoId = "";

    const rewriter = new HTMLRewriter()
      .on("entry", {
        element() {
          currentEntryVid = "";
          currentEntryTitle = "";
        },
      })
      .on("entry > yt\\:videoId, entry > videoId", {
        text(chunk) {
          currentEntryVid += chunk.text;
          if (chunk.lastInTextNode) {
            const vid = currentEntryVid.trim();
            if (vid && !firstVideoId) {
              firstVideoId = vid;
            }
          }
        },
      })
      .on("entry > title", {
        text(chunk) {
          currentEntryTitle += chunk.text;
          if (chunk.lastInTextNode) {
            const vid = currentEntryVid.trim();
            if (
              vid &&
              !matchedVideoId &&
              /hudson\s*river/i.test(currentEntryTitle)
            ) {
              matchedVideoId = vid;
            }
          }
        },
      });

    const transformed = rewriter.transform(resp);
    await transformed.body?.pipeTo(new WritableStream());

    const videoId = matchedVideoId || firstVideoId || DEFAULT_VIDEO_ID;

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

export default {
  async fetch(request: Request): Promise<Response> {
    const url = new URL(request.url);

    if (url.pathname === "/api/live") {
      if (request.method !== "GET" && request.method !== "HEAD") {
        return Response.json({ error: "Method not allowed" }, { status: 405 });
      }
      return handleLiveLookup();
    }

    if (url.pathname.startsWith("/api/")) {
      return Response.json({ error: "Not found" }, { status: 404 });
    }

    // Redirect mistyped non-asset paths back to the main live river view
    return Response.redirect(new URL("/", url.origin).toString(), 302);
  },
} satisfies ExportedHandler;
