// subagent_run.outcome is free text written at close-out ('done', 'success',
// 'clean - no regressions', 'partial - ...', 'reopened-fixed'). Classifying
// it is a prefix heuristic — kept in exactly one place.
export function outcomeLooksSuccessful(outcome: string): boolean {
  return /^(done|success|clean)/i.test(outcome.trim())
}

export function outcomeLooksFailed(outcome: string): boolean {
  return /^(fail|error|abort)/i.test(outcome.trim())
}
