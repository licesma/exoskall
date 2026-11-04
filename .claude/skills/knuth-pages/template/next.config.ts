import type { NextConfig } from 'next'

// Dev-server-only site: no output/build config, no webpack (Turbopack is the
// default), no telemetry (script sets NEXT_TELEMETRY_DISABLED=1). The dev
// overlay badge is hidden because the site renders inside the knuth iframe.
const nextConfig: NextConfig = {
  devIndicators: false,
}

export default nextConfig
