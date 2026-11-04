import 'server-only'

/** Run metrics are not part of the issue-only database. */
export function readRecentRuns(): never {
  throw new Error('Run metrics were retired; read issues for findings and outcomes.')
}
export function readRunTotals(): never {
  throw new Error('Run metrics were retired; read issues for findings and outcomes.')
}
