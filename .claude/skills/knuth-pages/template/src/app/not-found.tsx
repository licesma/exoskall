import Link from 'next/link'

export default function NotFound() {
  return (
    <main className="mx-auto flex max-w-5xl flex-col items-center gap-4 px-6 py-24 text-center">
      <h1 className="text-lg font-semibold">No such page</h1>
      <p className="text-sm text-muted-foreground">
        Nothing lives at this route.
      </p>
      <Link className="text-sm underline underline-offset-4" href="/">
        Back to the overview
      </Link>
    </main>
  )
}
