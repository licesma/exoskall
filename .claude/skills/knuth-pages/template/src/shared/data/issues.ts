import 'server-only'

import type { Issue } from '@/shared/lib/types'
import { withDb } from './database'

/** Human keys and statuses are preserved; parents resolve through stable IDs. */
export async function readIssues(): Promise<Issue[]> {
  return withDb((db) => db.prepare(Number(db.prepare('PRAGMA user_version').get()?.user_version) === 2 ? `SELECT id,title,description,status,mode,parent FROM issue ORDER BY id` : `
    SELECT i.display_key AS id, i.title, i.description, i.status, i.mode,
           p.display_key AS parent
    FROM issue i LEFT JOIN issue p ON p.id = i.parent_id
    ORDER BY i.updated_at DESC, i.rowid DESC
  `).all().map((row) => ({
    id: String(row.id),
    title: String(row.title),
    description: String(row.description ?? ''),
    status: String(row.status),
    mode: String(row.mode ?? 'unknown'),
    parent: row.parent === null ? null : String(row.parent),
  })))
}
