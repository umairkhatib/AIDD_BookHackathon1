---
id: 1
title: Update Constitution
stage: constitution
date: 2026-01-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.constitution
labels: [constitution,spec-driven,ai-book,rag-chatbot]
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

Project: AI-Spec–Driven Book with Embedded RAG Chatbot

Core principles:
- Specification-first, AI-native development
- Technical accuracy via primary sources
- Clarity for software engineers and AI practitioners
- Reproducibility and transparency
- Modular architecture (book, backend, chatbot)

Key standards:
- All claims traceable to official documentation or reputable sources
- Sources prioritized: official docs → industry standards → OSS repos
- Citations: Markdown footnotes with primary links
- All AI-generated content reviewed for hallucinations
- Security best practices for APIs, secrets, and data access

Book standards:
- Written in Docusaurus (MDX), deployed to GitHub Pages
- Clear navigation and structured chapters
- Each chapter includes: concepts, architecture, implementation, pitfalls
- Code examples must be complete, runnable, and version-pinned

RAG chatbot standards:
- Stack: OpenAI Agents/ChatKit, FastAPI, Neon Postgres, Qdrant Cloud
- Two modes: full-book context and user-selected text

## Response snapshot

Updated the project constitution with the specified principles and standards for the AI-Spec–Driven Book with Embedded RAG Chatbot project. The constitution now includes core principles for specification-first development, technical accuracy, clarity, reproducibility, modular architecture, and security best practices. It also defines technology standards for source documentation, book platform, and RAG chatbot stack, along with development workflow and governance guidelines.

## Outcome

- ✅ Impact: Updated constitution file with comprehensive principles and standards
- 🧪 Tests: none
- 📁 Files: .specify/memory/constitution.md
- 🔁 Next prompts: none
- 🧠 Reflection: Successfully transformed template constitution with project-specific content

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): none