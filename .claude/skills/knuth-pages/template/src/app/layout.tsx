import type { Metadata } from 'next'
import type { ReactNode } from 'react'
import './globals.css'

export const metadata: Metadata = {
  title: 'knuth pages',
  description: 'Live view of this repository',
}

// Light shell, matching knuth itself, on the system font stack
// (--font-sans in globals.css).
// No next/font: a Google-font fetch would make first compile
// network-dependent for an offline local tool.
export default function RootLayout({ children }: { children: ReactNode }) {
  return (
    <html lang="en">
      <body className="min-h-screen antialiased">{children}</body>
    </html>
  )
}
