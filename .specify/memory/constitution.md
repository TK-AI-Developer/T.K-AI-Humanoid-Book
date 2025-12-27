<!--
SYNC IMPACT REPORT:
Version change: N/A → 1.0.0
Modified principles: N/A (new constitution)
Added sections: All sections are new
Removed sections: N/A
Templates requiring updates: ✅ updated - .specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md
Follow-up TODOs: None
-->
# AI/Spec-Driven Technical Book with Embedded RAG Chatbot Constitution

## Core Principles

### Spec-first, implementation-second
No content or code without a spec

### Single source of truth
Docusaurus Markdown

### Grounded generation
No hallucinations, no unstated assumptions

### Modular and reproducible architecture
Modular and reproducible architecture

### Educational clarity for technical learners
Educational clarity for technical learners

### Content standards
All chapters derived from explicit `sp.specify` definitions; Each chapter must complete: concept → explanation → example → takeaway; Language: precise, instructional, technically accurate; No placeholders or partially written sections; Consistent terminology across all modules

## Technical constraints
Docs framework: Docusaurus (routing fixed at `/`); Deployment: GitHub Pages; Spec framework: Spec-Kit Plus; Code generation: Claude Code only; Backend: FastAPI; Vector DB: Qdrant Cloud (Free Tier); Relational DB: Neon Serverless Postgres; AI layer: OpenAI Agents / ChatKit SDKs; All book content in Markdown (.md / .mdx)

## RAG chatbot rules
Index only published Markdown content; Support: Full-book semantic Q&A, Q&A restricted to user-selected text only; Responses must cite or clearly reference source sections; No answers beyond retrieved context; Embedding pipeline must be fully reproducible

## Governance
Each module must pass: Spec completeness check, Implementation alignment check, Docusaurus render validation; Chatbot must pass: Book-wide queries, Section-specific queries, Selected-text-only queries; Success criteria: Book builds and deploys successfully, All modules are complete and spec-aligned, RAG chatbot is embedded and functional, Zero broken pages, empty sections

**Version**: 1.0.0 | **Ratified**: 2025-01-09 | **Last Amended**: 2025-01-09