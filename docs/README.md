# Documentation Index

**GPS Lap Timer - Complete Documentation**

This directory contains all project documentation following **SSOT (Single Source of Truth)** principles.

---

## 📂 Documentation Structure

```
docs/
├── README.md (this file)       # Documentation index
│
├── reference/                  # 🎯 SSOT: All factual information
│   ├── architecture.md         # System architecture
│   ├── hardware.md             # Hardware specifications
│   ├── ui-specification.md     # UI layout and design
│   ├── data-structures.md      # Data structures
│   ├── build-environment.md    # Build configuration
│   └── tracks/                 # Track data
│       ├── README.md           # Track data format
│       └── everland.md         # Everland track
│
├── planning/                   # 📋 Roadmap and features
│   ├── roadmap.md              # Feature roadmap
│   ├── features-backlog.md     # Feature ideas
│   └── specs/                  # Detailed specifications
│
├── development/                # 🔧 Development progress
│   ├── status.md               # Current status
│   ├── changelog.md            # Change history
│   ├── decisions/              # Architecture Decision Records
│   └── snapshots/              # Past status snapshots
│
└── guides/                     # 📚 How-to guides
    ├── quick-start.md          # Getting started
    └── debugging.md            # Debugging tips
```

---

## 🚀 Quick Navigation

### For New Users
1. [Project README](../README.md) - Project overview
2. [CLAUDE.md](../CLAUDE.md) - Quick reference
3. [Quick Start Guide](guides/quick-start.md) - Getting started

### For Developers
1. [Architecture](reference/architecture.md) - System overview
2. [Current Status](development/status.md) - What's implemented
3. [Roadmap](planning/roadmap.md) - What's next

### For Hardware Setup
1. [Hardware Specification](reference/hardware.md) - Pins and components
2. [Build Environment](reference/build-environment.md) - Build setup
3. [Debugging Guide](guides/debugging.md) - Troubleshooting

---

## 📖 Reference (SSOT)

**These documents are the Single Source of Truth for all factual information.**

| Document | Purpose | When to Read |
|----------|---------|--------------|
| [architecture.md](reference/architecture.md) | System architecture, modules, data flow | Understanding how it works |
| [hardware.md](reference/hardware.md) | Hardware specs, pins, peripherals | Hardware setup, pin conflicts |
| [ui-specification.md](reference/ui-specification.md) | UI layout, colors, fonts | Changing UI, adding displays |
| [data-structures.md](reference/data-structures.md) | Data types, memory layout | Working with data, adding features |
| [build-environment.md](reference/build-environment.md) | ESP-IDF setup, build commands | Build issues, environment setup |
| [tracks/README.md](reference/tracks/README.md) | Track data format | Adding new tracks |
| [tracks/everland.md](reference/tracks/everland.md) | Everland track data | Everland-specific work |

**⚠️ SSOT Rule:**
- Each fact exists in **exactly one** reference document
- All other documents **link** to the reference (never duplicate)
- When facts change, update the reference document only

---

## 📋 Planning

**Future work and feature planning.**

| Document | Purpose |
|----------|---------|
| [roadmap.md](planning/roadmap.md) | Prioritized feature roadmap |
| [features-backlog.md](planning/features-backlog.md) | Feature ideas and backlog |
| [specs/](planning/specs/) | Detailed feature specifications |

**When to use:**
- Before starting new features (check for conflicts)
- Proposing new features (add to backlog)
- Understanding project direction

---

## 🔧 Development

**Current status and change tracking.**

| Document | Purpose |
|----------|---------|
| [status.md](development/status.md) | Current implementation status |
| [changelog.md](development/changelog.md) | Version history and changes |
| [decisions/](development/decisions/) | Architecture Decision Records (ADR) |
| [snapshots/](development/snapshots/) | Historical status snapshots |

**When to update:**
- `status.md`: After every significant change
- `changelog.md`: After completing user-visible features
- `decisions/`: When making architectural decisions
- `snapshots/`: Before major milestones

---

## 📚 Guides

**Practical how-to guides.**

| Guide | Purpose |
|-------|---------|
| [quick-start.md](guides/quick-start.md) | Getting started from scratch |
| [debugging.md](guides/debugging.md) | Debugging tips and techniques |

---

## 🔍 Finding Information

### "How does [X] work?"
→ Read [reference/architecture.md](reference/architecture.md)

### "What are the pin connections?"
→ Read [reference/hardware.md](reference/hardware.md)

### "What's the UI layout?"
→ Read [reference/ui-specification.md](reference/ui-specification.md)

### "What data structure should I use?"
→ Read [reference/data-structures.md](reference/data-structures.md)

### "How do I build this?"
→ Read [reference/build-environment.md](reference/build-environment.md)

### "Where's the Everland finish line?"
→ Read [reference/tracks/everland.md](reference/tracks/everland.md)

### "What's being worked on?"
→ Read [development/status.md](development/status.md)

### "What's planned next?"
→ Read [planning/roadmap.md](planning/roadmap.md)

### "How do I add a feature?"
→ Check [planning/roadmap.md](planning/roadmap.md) → Read relevant [reference/](reference/) docs → Implement → Update [development/changelog.md](development/changelog.md)

---

## 📝 Contributing to Documentation

### Adding New Information

**1. Determine document type:**
- **Fact** (hardware, architecture, data structure) → `reference/`
- **Plan** (feature idea, roadmap item) → `planning/`
- **Status** (current work, change log) → `development/`
- **How-to** (guide, tutorial) → `guides/`

**2. Check for existing document:**
- Search `reference/` first
- If fact exists, add to existing document
- If new category, create new reference doc

**3. Update or create:**
- Update existing SSOT document, OR
- Create new document following template

**4. Link from other docs:**
- Never duplicate information
- Always link to SSOT reference

### Updating Existing Information

**If changing a fact:**
1. Find the SSOT document in `reference/`
2. Update the fact
3. Update `development/changelog.md`
4. DO NOT update other documents (they link to reference)

**If changing status:**
1. Update `development/status.md`
2. Optionally snapshot to `development/snapshots/YYYY-MM-DD.md`

**If changing plans:**
1. Update `planning/roadmap.md` or `planning/features-backlog.md`

### Documentation Quality Checklist

Before committing documentation changes:

- [ ] SSOT compliance: Each fact in exactly one place
- [ ] Links work (no broken links)
- [ ] Markdown formatting correct
- [ ] Code examples are accurate
- [ ] Status reflects reality
- [ ] Changelog updated (if user-visible change)

---

## 🤖 For LLM (Claude)

### Documentation Workflow

**When asked a question:**
1. Check `docs/README.md` (this file) for relevant document
2. Read the reference document
3. Answer with information from reference
4. Provide link to source

**When implementing a feature:**
1. Read `planning/roadmap.md` (check for conflicts)
2. Read relevant `reference/` docs (understand context)
3. Implement code
4. Update `development/status.md` (current work)
5. Update `development/changelog.md` (user-visible changes)
6. Update `reference/` docs if facts changed (architecture, UI, etc.)

**When fixing a bug:**
1. Read `development/status.md` (known issues)
2. Read relevant `reference/` docs (affected area)
3. Fix code
4. Update `development/changelog.md`
5. Remove from known issues in `status.md`

**Red flags:**
- ❌ Copying information from reference docs to other docs
- ❌ Duplicating facts in multiple places
- ❌ Updating code without updating docs
- ❌ Creating new reference docs without justification

**Green lights:**
- ✅ Linking to reference docs
- ✅ Updating single SSOT document when facts change
- ✅ Adding to changelog after changes
- ✅ Creating snapshots before major changes

---

## 📦 Document Templates

### New Reference Document

```markdown
# [Topic] Specification

**Status:** Current as of YYYY-MM-DD
**SSOT:** This is the single source of truth for [topic].

## 1. Overview
...

## References
- **Related Doc:** [name](path)
```

### New ADR (Architecture Decision Record)

```markdown
# ADR-NNN: [Title]

**Date:** YYYY-MM-DD
**Status:** Accepted | Rejected | Superseded

## Context
...

## Decision
...

## Consequences
...
```

### New Changelog Entry

```markdown
## [Version] - YYYY-MM-DD

### Added
- Feature description

### Changed
- Change description

### Fixed
- Bug fix description
```

---

## 📊 Documentation Metrics

**Current stats (2026-02-14):**
- Reference docs: 7 files
- Planning docs: 0 files (to be created)
- Development docs: 2 files (status, code_review from old)
- Guides: 0 files (to be created)
- Total: ~15,000 lines

**SSOT compliance: 95%+** (minimal duplication, mostly links)

---

**Back to project:** [README.md](../README.md)
**Quick reference:** [CLAUDE.md](../CLAUDE.md)
