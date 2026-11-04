import 'server-only'

import { join } from 'node:path'
import { DatabaseSync } from 'node:sqlite'
import { repoRoot } from './repo-root'

/** Read on the site's own host. Never create a database or use legacy files. */
export function withDb<T>(read: (db: DatabaseSync) => T): T {
  const db = new DatabaseSync(join(repoRoot, '.knuth', 'issues.db'), { readOnly: true })
  try {
    db.exec('PRAGMA query_only = ON; BEGIN')
    const version = db.prepare('PRAGMA user_version').get()
    if (![1, 2].includes(Number(version?.user_version))) throw new Error('Unsupported Knuth database schema')
    return read(db)
  } finally {
    db.close()
  }
}
