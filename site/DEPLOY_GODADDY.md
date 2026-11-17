# `hudsonriver.live` — Cloudflare + GoDaddy Setup

`site/` is deployed as a Cloudflare Worker + Static Assets app (`hudsonriver-live`) using the official `cf` CLI ([cloudflare.config.ts](cloudflare.config.ts) + [src/index.ts](src/index.ts)).

---

## 1. Finish Nameserver Switch in GoDaddy (One-Time)
Your Cloudflare zone (`hudsonriver.live`, Zone ID `d622113a318523c1e03979257d723067`) and custom domain bindings (`hudsonriver.live` + `www.hudsonriver.live`) are already created and waiting on GoDaddy delegation:

1. Open [GoDaddy Domain Settings for `hudsonriver.live`](https://dcc.godaddy.com/manage/hudsonriver.live/dns) → **Nameservers** tab → **Change Nameservers**.
2. Choose **I'll use my own nameservers** and replace `ns41.domaincontrol.com` / `ns42.domaincontrol.com` with:
   - `ariadne.ns.cloudflare.com`
   - `zeus.ns.cloudflare.com`
3. Click **Save**. Once GoDaddy propagates the nameserver change (typically 5–30 minutes), Cloudflare automatically activates the zone and provisions SSL certificates for `https://hudsonriver.live` and `https://www.hudsonriver.live`.

---

## 2. Local Development & Redeploying Changes
Whenever you edit files in `site/public/` or `site/src/index.ts`:

```bash
cd "/Users/craignm/Library/CloudStorage/Dropbox-Dropbox/Craig Nevill-Manning/AI/PTZ camera over the Hudson River/site"

# Run local Cloudflare Workers runtime preview
cf dev

# Build and deploy to https://hudsonriver.live
cf deploy
```

Check whether GoDaddy's nameserver delegation has finished activating on Cloudflare at any time with:

```bash
cf zones get --zone hudsonriver.live
```
