# Document Audit Sync Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Align stale repository documents with the actual codebase state and add one canonical follow-up document for the remaining unfinished work.

**Architecture:** Update the small set of status-heavy documents that are currently misleading, instead of trying to rewrite every historical note. Then add one new canonical document under `docs/` that lists true unfinished tasks, document-mentioned but unimplemented features, and missing implementation steps or dependencies.

**Tech Stack:** Markdown, ROS1 catkin workspace documentation, repository source-of-truth verification via ripgrep/sed

---

### Task 1: Sync stale status documents

**Files:**
- Modify: `HANDOFF.md`
- Modify: `docs/plans/未完成计划总结报告.md`
- Modify: `docs/README.md` (if references are stale)
- Modify: `README.md` (if references are stale)

**Step 1:** Mark items that are now implemented as completed or partially completed.

**Step 2:** Preserve historical context where useful, but explicitly call out document/code drift.

**Step 3:** Fix broken or stale document references that would mislead future work.

### Task 2: Add one canonical remaining-work document

**Files:**
- Create: `docs/remaining_work_audit_2026-03-06.md`

**Step 1:** Add section for truly unfinished tasks.

**Step 2:** Add section for features mentioned in docs but not implemented in the repo.

**Step 3:** Add section for missing implementation steps, data, tooling, validation, and external dependencies.

**Step 4:** Add references to the exact source documents and representative code locations.

### Task 3: Verification

**Files:**
- Verify modified markdown files only

**Step 1:** Re-read all edited documents for internal consistency.

**Step 2:** Confirm key references still exist in the repo.

**Step 3:** Summarize the new canonical source of truth in the final response.
