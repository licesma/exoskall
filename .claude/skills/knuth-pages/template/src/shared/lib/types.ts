// Shared domain types. Every value crossing the server->client boundary is a
// plain serializable object: strings, numbers, booleans, null, arrays, plain
// objects. Dates travel as ISO strings.

export type Issue = {
  id: string
  title: string
  description: string
  status: string
  mode: string
  parent: string | null
}

export type SubagentRun = {
  id: number
  issue: string
  finishedAt: string
  durationMs: number
  tokens: number
  toolUses: number
  tier: string
  model: string
  outcome: string
  mode: string
}

export type RunTotals = {
  runs: number
  successes: number
  totalTokens: number
  medianDurationMs: number
}

export type Commit = {
  hash: string
  author: string
  /** ISO 8601 author date. */
  date: string
  subject: string
}
