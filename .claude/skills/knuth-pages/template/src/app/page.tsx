import { formatDistanceToNow } from 'date-fns'
import { readRecentCommits } from '@/shared/data/git'
import { readIssues } from '@/shared/data/issues'
import { repoName } from '@/shared/data/repo-root'
import type { Issue } from '@/shared/lib/types'
import { Badge } from '@/shared/ui/badge'
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from '@/shared/ui/card'
import { EmptyState } from '@/shared/ui/empty-state'
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from '@/shared/ui/table'

const statusVariant = (status: Issue['status']) =>
  status === 'done' ? 'secondary' : status === 'in-progress' ? 'default' : 'outline'

const ago = (iso: string) => {
  const date = new Date(iso)
  return Number.isNaN(date.getTime())
    ? iso
    : formatDistanceToNow(date, { addSuffix: true })
}

// The template's proof page: one server component reading the live repo
// through all three shared/data readers and composing shared/ui only
// (no _components at the app root — see the LAW).
export default async function HomePage() {
  const [issues, commits] = await Promise.all([readIssues(), readRecentCommits(10)])

  return (
    <main className="mx-auto flex max-w-5xl flex-col gap-6 px-6 py-10">
      <header className="flex items-baseline justify-between">
        <h1 className="text-xl font-semibold tracking-tight">{repoName}</h1>
        <p className="text-sm text-muted-foreground">
          live from the repository, on every request
        </p>
      </header>

      <Card>
        <CardHeader>
          <CardTitle>Issues</CardTitle>
          <CardDescription>
            {issues.length} issues
          </CardDescription>
        </CardHeader>
        <CardContent>
          {issues.length === 0 ? (
            <EmptyState
              title="No issues yet"
              hint="Create an issue with the Knuth database CLI."
            />
          ) : (
            <Table>
              <TableHeader>
                <TableRow>
                  <TableHead>Id</TableHead>
                  <TableHead>Title</TableHead>
                  <TableHead>Status</TableHead>
                  <TableHead>Mode</TableHead>
                </TableRow>
              </TableHeader>
              <TableBody>
                {issues.slice(0, 12).map((issue) => (
                  <TableRow key={issue.id}>
                    <TableCell className="font-mono text-xs">{issue.id}</TableCell>
                    <TableCell>{issue.title}</TableCell>
                    <TableCell>
                      <Badge variant={statusVariant(issue.status)}>{issue.status}</Badge>
                    </TableCell>
                    <TableCell className="text-muted-foreground">{issue.mode}</TableCell>
                  </TableRow>
                ))}
              </TableBody>
            </Table>
          )}
        </CardContent>
      </Card>

      <Card>
        <CardHeader>
          <CardTitle>Recent commits</CardTitle>
          <CardDescription>git log, newest first</CardDescription>
        </CardHeader>
        <CardContent>
          {commits.length === 0 ? (
            <EmptyState
              title="No commits"
              hint="This directory is not a git repository yet."
            />
          ) : (
            <ul className="flex flex-col gap-2">
              {commits.map((commit) => (
                <li key={commit.hash} className="flex items-baseline gap-3 text-sm">
                  <code className="shrink-0 font-mono text-xs text-muted-foreground">
                    {commit.hash.slice(0, 7)}
                  </code>
                  <span className="truncate">{commit.subject}</span>
                  <span className="ml-auto shrink-0 text-xs text-muted-foreground">
                    {commit.author}, {ago(commit.date)}
                  </span>
                </li>
              ))}
            </ul>
          )}
        </CardContent>
      </Card>
    </main>
  )
}
