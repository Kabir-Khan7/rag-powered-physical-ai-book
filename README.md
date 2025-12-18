# Hackathon I – Physical AI Textbook Platform

This monorepo houses the full stack needed to publish the Physical AI & Humanoid Robotics textbook, power retrieval-augmented question answering, and deliver personalized learning experiences.

## Repository Layout

| Path | Purpose |
| --- | --- |
| `Book/book.md` | Single-source manuscript (do not edit generated docs manually). |
| `scripts/` | Automation helpers (`split_book.py`, `ingest_book.py`). |
| `website/` | Docusaurus docs-only frontend deployed to Vercel. |
| `backend/` | FastAPI service powering QA, ingestion, translation, personalization. |

## Frontend (Phase 1)

The Docusaurus site is configured in docs-only mode where Chapter 1 serves the homepage and client-side search is available offline. All chapters are generated from `Book/book.md` via `python scripts/split_book.py`.

### Local Development

```bash
python scripts/split_book.py          # regenerate chapter files if book.md changes
npm --prefix website install          # install frontend deps
npm --prefix website run start        # dev server at http://localhost:3000
npm --prefix website run build        # production build (outputs to website/build)
```

Set `BACKEND_URL` to your FastAPI server in `website/.env` (copy from `.env.example`) so the chatbot widget can reach the API.

## Deployment (Phase 2)

Vercel deployment is configured through `vercel.json` to run the website build. Set the project root to the repo’s root when importing on Vercel. The resulting public URL should be documented here once provisioned.

- **Build command**: `npm --prefix website run build`
- **Install command**: `npm --prefix website install`
- **Output directory**: `website/build`
- **Live URL**: _pending deployment (update here after Vercel import)_

## Content Ingestion (Phase 3)

`scripts/ingest_book.py` reads every generated chapter, chunks content (~800 tokens, 150 overlap), generates embeddings with OpenAI, and upserts them into Qdrant with metadata (`chapter_id`, `heading`, `source_url`, `snippet`).

### Required Environment

```
OPENAI_API_KEY=...
OPENAI_EMBEDDING_MODEL=text-embedding-3-large  # optional override
QDRANT_URL=...
QDRANT_API_KEY=...
QDRANT_COLLECTION=physical-ai-textbook
SITE_BASE_URL=https://physical-ai-textbook.vercel.app  # optional override
```

### Run

```bash
export OPENAI_API_KEY=...
export QDRANT_URL=...
export QDRANT_API_KEY=...
export QDRANT_COLLECTION=physical-ai-textbook
python scripts/ingest_book.py
```

The script is idempotent: chunk IDs are deterministic so re-runs safely overwrite prior vectors.

## Backend (Phase 4)

FastAPI app lives under `backend/app`. Key endpoints:

| Endpoint | Description |
| --- | --- |
| `POST /api/qa` | QA with optional `selected_text` constraint, always returns citations. |
| `POST /api/ingest` | Calls the ingestion pipeline via Python to refresh Qdrant. |
| `POST /api/translate` | English → Urdu translation via OpenAI, preserving technical structure. |
| `POST /api/personalize` | Generates overlay notes tailored to learner preferences. |

### Backend Setup

```bash
cd backend
python -m venv .venv
.venv\Scripts\activate  # or source .venv/bin/activate
pip install -r requirements.txt
uvicorn backend.app.main:app --reload
```

The backend reads the same environment variables listed above plus:

```
OPENAI_RESPONSE_MODEL=gpt-4o-mini  # optional override
SITE_BASE_URL=https://physical-ai-textbook.vercel.app
NEON_DATABASE_URL=postgresql+asyncpg://...   # required for later phases
BETTER_AUTH_SECRET=...                      # required for later phases
```

## Chatbot UI (Phase 5)

A floating widget (bottom-right) lets readers ask questions or toggle "Use selected text only". Selecting text in the doc and clicking "Capture selection" populates the constraint. Responses show clickable citations that jump to the referenced chapter section. Configure `BACKEND_URL` so the widget can reach `/api/qa`.

## Authentication & Personalization (Phases 6-7)

- Better-Auth Python SDK proxies signup/signin/session validation (see `BetterAuthService`).
- User profiles are persisted in Neon (`backend/app/db_models.py`) capturing software background, hardware background, and learning preference.
- Endpoints:
  - `POST /api/auth/signup`
  - `POST /api/auth/signin`
  - `GET /api/auth/session`
  - `POST /api/profile` & `GET /api/profile`
- Frontend adds a floating auth panel plus a "Personalize this chapter" button (visible only when logged in) that calls `/api/personalize` with the stored profile metadata.

### Additional Environment

```
NEON_DATABASE_URL=postgresql+asyncpg://<user>:<password>@<host>/<database>
BETTER_AUTH_SECRET=<Better-Auth admin token>
BETTER_AUTH_HOST=https://api.better-auth.com  # override if self-hosting
```

- Chapter footers now include a “Translate to Urdu” toggle that calls `/api/translate` with `chapter_id` and displays a right-to-left panel while keeping English available.

## Next Steps

1. Deploy backend + Qdrant + Neon + Vercel and capture live URLs along with env var instructions (Phase 9).
2. End-to-end validation checklist plus demo steps in this README.
