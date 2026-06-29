# Roadmap

## Vision

Keep `apollo_perception_ros` useful as an **educational reference** for Apollo 3.0
perception — with honest documentation, triaged issues, and incremental
compatibility improvements where feasible.

## 2026 Q3 — Maintenance restart (Phase 1)

- [x] Add Apache-2.0 LICENSE, NOTICE, third-party notices
- [x] Reposition README (status, audience, limitations)
- [x] Add CONTRIBUTING.md and SECURITY.md
- [x] Triage open issues and add labels
- [ ] Publish `v0.1.0-historical` release
- [ ] Document demo bag acquisition and playback
- [ ] Publish supported environment matrix

## 2026 Q4 — Documentation & CI (Phase 2)

- [ ] Add issue/PR templates
- [ ] Fix broken documentation links
- [ ] Add lightweight CI (markdown / shellcheck / repo structure)
- [ ] Consolidate CUDA / Docker troubleshooting guide

## Future (best-effort)

- [ ] Ego motion compensation
- [ ] Sequence type fuser
- [ ] Community sensor adapter examples
- [ ] Evaluate modernized Docker base image (no guarantee of model compatibility)

## Out of scope

- Full migration to Apollo Cyber RT
- Production deployment guarantees
- Supporting every GPU generation without community help

## Codex-assisted maintenance

Maintainer workflows where AI assistance adds value:

- Issue triage and duplicate detection
- Drafting compatibility documentation from error logs
- PR review for docs and small fixes
- Security / dependency hygiene scans
- Generating reproducible Docker troubleshooting steps

API credits (if granted via Codex for OSS) will be used **only** for these OSS
maintenance workflows.
