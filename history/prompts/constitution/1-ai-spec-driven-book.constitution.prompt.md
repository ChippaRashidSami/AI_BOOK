---
id: 1
title: AI Spec Driven Book Constitution
stage: constitution
date: 2025-12-16
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: sp.constitution
labels: [constitution,spec-first,rag-chatbot,ai-authorship]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

```text
                          Project: AI-Spec–Driven Book with Embedded RAG Chatbot

Mission:
Author and deploy a technical book using Spec-Kit Plus and Claude Code, and embed
a production-grade RAG chatbot that answers questions about the book content,
including user-selected text only.

Core principles:
- Spec-first development (Spec-Kit Plus governs all behavior)
- AI-native authorship (Claude Code as primary author)
- Verifiable, non-hallucinatory content
- Fully reproducible builds and deployment

Deliverables:
1. Book
   - Written with Docusaurus using Spec-Kit Plus specs
   - Authored via Claude Code
   - Deployed to GitHub Pages

2. RAG Chatbot
   - Embedded in the book UI
   - FastAPI backend
   - OpenAI Agents / ChatKit SDKs
   - Neon Serverless Postgres (metadata)
   - Qdrant Cloud (vector search)
   - Supports:
     - Full-book Q&A
     - Q&A constrained to user-selected text
     - Refusal on insufficient context

Standards:
- Modular, documented architecture
- Secure key management
- Logged and observable chatbot behavior
- Clear separation of specs, content, and code

Constraints:
- Use Spec-Kit Plus as the source of truth
- Claude Code only for AI authoring
- Free-tier services where specified
- GitHub Pages for deployment

Success criteria:
- Book publicly accessible on GitHub Pages
- Embedded chatbot answers accurately from retrieved context only
- Selected-text–only queries enforced
- Zero undocumented behavior; full spec compliance
- live book on github pages. fullu functional RAG chat bot . all specs implemented via spec kit plus """
```

## Response snapshot

Updated the project constitution file (.specify/memory/constitution.md) with project-specific principles for the AI-Spec–Driven Book with Embedded RAG Chatbot project. The new constitution includes six core principles: Spec-First Development, AI-Native Authorship, Verifiable Non-Hallucinatory Content, Production-Grade Architecture, Reproducible Builds and Deployment, and Selected-Text-Only Queries Enforcement. Additional sections cover Technology Stack Requirements, Development Workflow, and Governance rules.

## Outcome

- ✅ Impact: Created project-specific constitution with 6 core principles and governance rules
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with project-specific values
- 🔁 Next prompts: Will implement the book and RAG chatbot following these constitutional principles
- 🧠 Reflection: Constitution now properly reflects the requirements for an AI-authored, spec-driven book with embedded RAG chatbot

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin implementing the book structure following Docusaurus and the constitutional principles