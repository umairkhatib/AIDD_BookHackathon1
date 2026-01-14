<!-- SYNC IMPACT REPORT
Version change: N/A -> 1.0.0
Modified principles: N/A (new constitution)
Added sections: All sections based on project requirements
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md ✅ updated
  - .specify/templates/spec-template.md ✅ updated
  - .specify/templates/tasks-template.md ✅ updated
  - .specify/templates/commands/*.md ⚠ pending
Follow-up TODOs: None
-->

# AI-Spec–Driven Book with Embedded RAG Chatbot Constitution

## Core Principles

### Specification-First, AI-Native Development
Specification-first, AI-native development: All development begins with clear, detailed specifications before implementation; AI tools are integrated throughout the development lifecycle to enhance productivity and maintain quality; Development follows a structured, predictable process that leverages AI assistance for code generation, testing, and documentation.

### Technical Accuracy via Primary Sources
Technical accuracy via primary sources: All claims must be traceable to official documentation or reputable sources; Information is prioritized by source reliability: official docs → industry standards → open-source repositories; Citations use Markdown footnotes with direct links to primary sources; All AI-generated content undergoes hallucination review to ensure factual accuracy.

### Clarity for Software Engineers and AI Practitioners
Clarity for software engineers and AI practitioners: Documentation and code must be accessible to both traditional software engineers and AI practitioners; Concepts are explained with both theoretical foundations and practical implementation details; Clear navigation and structured chapters guide users through complex topics systematically.

### Reproducibility and Transparency
Reproducibility and transparency: All processes, builds, and deployments must be reproducible from documentation alone; Code examples are complete, runnable, and version-pinned to ensure consistent results; All dependencies and configurations are explicitly declared and documented; Development environments are standardized and documented.

### Modular Architecture
Modular architecture: System components (book, backend, chatbot) are designed as independent modules with well-defined interfaces; Each module can be developed, tested, and deployed separately while maintaining integration capabilities; Clear separation of concerns ensures maintainability and scalability; Architecture supports both monolithic and microservice deployment patterns.

### Security Best Practices
Security best practices for APIs, secrets, and data access: All API endpoints implement proper authentication and authorization; Secrets are never hardcoded and stored securely using environment variables or secret management systems; Data access patterns follow the principle of least privilege; All security-sensitive operations are logged and monitored for audit purposes.

## Technology Standards

### Source Documentation and Verification
All claims traceable to official documentation or reputable sources: Information sources are prioritized as official docs → industry standards → open-source repositories; Citations use Markdown footnotes with primary links; All AI-generated content is reviewed for hallucinations; Technical accuracy verified through primary sources.

### Book Platform Standards
Book written in Docusaurus (MDX), deployed to GitHub Pages: Clear navigation and structured chapters with consistent formatting; Each chapter includes concepts, architecture, implementation, and pitfalls sections; Code examples are complete, runnable, and version-pinned; Navigation follows logical progression for learning objectives.

### RAG Chatbot Stack Standards
RAG chatbot stack uses OpenAI Agents/ChatKit, FastAPI, Neon Postgres, Qdrant Cloud: Two operational modes supported - full-book context and user-selected text; Backend services follow RESTful API design principles; Database schema supports efficient vector search and retrieval operations; Vector storage optimized for document chunking and similarity search.

## Development Workflow

### Implementation Standards
All code changes follow specification-driven development: Features implemented only after detailed specifications are approved; Code reviews verify compliance with architectural principles; Automated testing validates both functionality and adherence to standards; Peer review process includes verification of source citations and technical accuracy.

### Quality Assurance
Quality gates include source verification, technical accuracy, and security compliance: All claims must have verifiable sources; Code passes automated security scanning; Performance benchmarks meet established requirements; Documentation completeness verified before merge.

## Governance

Constitution governs all development activities and supersedes informal practices: All team members must familiarize themselves with these principles; Amendments require documentation of rationale and impact assessment; Regular compliance reviews ensure adherence to standards; Version control tracks all changes to this constitution.

**Version**: 1.0.0 | **Ratified**: 2026-01-09 | **Last Amended**: 2026-01-09
