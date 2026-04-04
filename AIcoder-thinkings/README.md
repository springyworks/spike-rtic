# AIcoder-thinkings — AI Agent Memory & Reasoning Logs

This folder stores AI agent memory files and reasoning logs from the
MCP-assisted development workflow.  These are **learning artifacts** —
snapshots of rules, patterns, and insights that the AI accumulated
during firmware development.

## Purpose

- **Persistent memory:** Copilot's memory files (`/memories/`) live in
  a VS Code config path and can be lost on reinstall.  This folder
  keeps version-controlled copies.
- **Learning value:** Past reasoning, mistakes, and corrections are
  valuable for understanding why code is the way it is.
- **Continuity:** New sessions start faster when the AI has context
  from previous work.

## Structure

| File | Content |
| ---- | ------- |
| `spike-rules.md` | Core project rules (10+ rules, never violate) |
| `session-notes/` | Per-session working notes (task context, progress) |
| `reasoning-logs/` | Notable reasoning chains and decisions |

## Syncing with Copilot Memory

The AI's live memory is at `/memories/spike-rules.md` (VS Code internal).
Periodically copy updated rules here to keep them in the repo.  The
canonical source for rules is always the one in Copilot memory — this
folder is the archive.
