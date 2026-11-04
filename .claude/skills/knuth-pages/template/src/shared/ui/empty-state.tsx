import { cn } from '@/shared/lib/utils'

// The house empty-state idiom: when a reader legitimately returns nothing
// (no issues yet, no stats DB, not a git repo), render this instead of an
// empty table — never a blank region, never a thrown error.
export function EmptyState({
  title,
  hint,
  className,
}: {
  title: string
  hint?: string
  className?: string
}) {
  return (
    <div
      className={cn(
        'flex flex-col items-center justify-center gap-1 rounded-lg border border-dashed px-6 py-10 text-center',
        className,
      )}
    >
      <p className="text-sm font-medium">{title}</p>
      {hint ? <p className="text-xs text-muted-foreground">{hint}</p> : null}
    </div>
  )
}
