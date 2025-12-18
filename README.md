# Hackathon I – Physical AI Textbook Platform

This repo packages the entire Physical AI & Humanoid Robotics textbook workflow: chapter generation, Docusaurus frontend, RAG-ready ingestion, and a FastAPI backend for QA, personalization, and Urdu translation.

## Repository Layout

| Path | Purpose |
| --- | --- |
| `Book/book.md` | Single-source manuscript (do not edit generated docs manually). |
| `scripts/split_book.py` | Splits the book into per-chapter docs with metadata. |
| `scripts/ingest_book.py` | Chunks chapters, embeds with OpenAI, and upserts into Qdrant. |
| `website/` | Docusaurus docs-only site deployed to Vercel. |
| `backend/` | FastAPI service handling QA, ingestion, translation, personalization. |

---

## Frontend (Docusaurus)

Docs-only Docusaurus is configured so Chapter 1 renders at `/` and local search works offline.

```bash
python scripts/split_book.py          # regenerate docs after editing Book/book.md
npm --prefix website install
npm --prefix website run start        # http://localhost:3000
npm --prefix website run build        # outputs to website/build
```

Set `website/.env` (copy from `.env.example`) so `BACKEND_URL` points to your FastAPI server.

### Deployment

`vercel.json` already specifies:
- **Install** `npm --prefix website install`
- **Build** `npm --prefix website run build`
- **Output** `website/build`

After Vercel import, record the live URL here.

---

## Content Ingestion

`scripts/ingest_book.py` performs:
1. Iterate every generated chapter (`website/docs/chapters/*.md`).
2. Chunk text (~800 tokens, 150 overlap) using `tiktoken` when available.
3. Embed chunks via OpenAI and upsert them into Qdrant with metadata (`chapter_id`, `heading`, `snippet`, `source_url`).

### Required Environment

```
OPENAI_API_KEY=...
OPENAI_EMBEDDING_MODEL=text-embedding-3-large  # optional override
QDRANT_URL=...
QDRANT_API_KEY=...
QDRANT_COLLECTION=physical-ai-textbook
SITE_BASE_URL=https://physical-ai-textbook.vercel.app  # optional override
```

Run:
```bash
python scripts/ingest_book.py
```
Re-running is safe—the script uses deterministic IDs and overwrites existing vectors.

---

## Backend (FastAPI)

Endpoints:
| Endpoint | Description |
| --- | --- |
| `POST /api/qa` | Answers questions with optional `selected_text`; always cites sources. |
| `POST /api/ingest` | Re-runs the ingestion script from within the API. |
| `POST /api/translate` | English → Urdu translation using OpenAI, accepts `text` or `chapter_id`. |
| `POST /api/personalize` | Generates overlays based on user-supplied learning preferences. |

### Setup

```bash
cd backend
python -m venv .venv
.venv\Scripts\activate  # or source .venv/bin/activate
pip install -r requirements.txt
uvicorn backend.app.main:app --reload --port 8000
```

Environment variables:
```
OPENAI_API_KEY=...
OPENAI_RESPONSE_MODEL=gpt-4o-mini   # optional override
OPENAI_EMBEDDING_MODEL=text-embedding-3-large
QDRANT_URL=...
QDRANT_API_KEY=...
QDRANT_COLLECTION=physical-ai-textbook
SITE_BASE_URL=https://physical-ai-textbook.vercel.app  # used for citations
```

FastAPI docs are available at `http://localhost:8000/docs`.

---

## Frontend Features

- **Chatbot widget** (floating button): asks book-wide questions or constrains answers to highlighted text via `/api/qa`.
- **Personalize panel** (per chapter footer): readers enter optional background + preferences, the frontend calls `/api/personalize` and displays the overlay inline.
- **Urdu translation toggle**: calls `/api/translate` with the chapter ID and shows right-to-left output while keeping the English original.

---

## End-to-End Checklist

1. `python scripts/split_book.py`
2. `npm --prefix website run build`
3. Configure backend `.env` and run `uvicorn backend.app.main:app --reload`
4. `python scripts/ingest_book.py`
5. Visit `http://localhost:3000`:
   - Ask chatbot question (with & without selected text)
   - Submit personalization form → overlay appears
   - Toggle Urdu translation
6. Deploy backend + frontend (Vercel) and update this README with live URLs + env instructions.
