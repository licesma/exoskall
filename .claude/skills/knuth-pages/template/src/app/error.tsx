'use client'

import { Button } from '@/shared/ui/button'

// Route error boundary (Next requires 'use client' here). Readers degrade to
// empty results for missing sources, so anything landing here is a real bug —
// show the message, offer a retry.
export default function ErrorBoundary({
  error,
  reset,
}: {
  error: Error & { digest?: string }
  reset: () => void
}) {
  return (
    <main className="mx-auto flex max-w-5xl flex-col items-center gap-4 px-6 py-24 text-center">
      <h1 className="text-lg font-semibold">Something broke while rendering</h1>
      <p className="max-w-prose text-sm text-muted-foreground">{error.message}</p>
      <Button variant="outline" onClick={() => reset()}>
        Try again
      </Button>
    </main>
  )
}
