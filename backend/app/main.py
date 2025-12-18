from __future__ import annotations

import os
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware

from backend.app import models
from backend.app.config import get_settings
from backend.app.services.rag import RAGService
from scripts.ingest_book import ingest_book

settings = get_settings()
app = FastAPI(
    title="Physical AI Backend",
    version="1.0.0",
    description="FastAPI service for textbook QA, ingestion, translation, and personalization.",
)
rag_service = RAGService(settings)

allowed_origins = [
    settings.site_base_url,
    "http://localhost:3000",
    "http://127.0.0.1:3000",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=allowed_origins,
    allow_methods=["*"],
    allow_headers=["*"],
)

ROOT = Path(__file__).resolve().parents[2]
CHAPTER_DIR = ROOT / "website" / "docs" / "chapters"


def load_chapter(chapter_id: str) -> str:
    path = CHAPTER_DIR / f"{chapter_id}.md"
    if not path.exists():
        raise HTTPException(status_code=404, detail=f"Chapter {chapter_id} not found.")
    text = path.read_text(encoding="utf-8")
    if text.startswith("---"):
        try:
            _, _, remainder = text.split("---", 2)
            text = remainder.strip()
        except ValueError:
            pass
    return text


@app.post("/api/qa", response_model=models.QAResponse)
def qa_endpoint(payload: models.QARequest):
    try:
        return rag_service.answer(payload)
    except Exception as exc:  # pragma: no cover
        raise HTTPException(status_code=500, detail=f"QA failed: {exc}") from exc


@app.post("/api/ingest", response_model=models.IngestResponse)
def ingest_endpoint():
    try:
        os.environ.setdefault("SITE_BASE_URL", settings.site_base_url)
        count = ingest_book()
        return models.IngestResponse(chunks_ingested=count, collection=settings.qdrant_collection)
    except Exception as exc:
        raise HTTPException(status_code=500, detail=f"Ingestion failed: {exc}") from exc


@app.post("/api/translate", response_model=models.TranslateResponse)
def translate_endpoint(payload: models.TranslateRequest):
    try:
        source_text = payload.text
        if payload.chapter_id:
            source_text = load_chapter(payload.chapter_id)
        if not source_text:
            raise HTTPException(status_code=400, detail="Nothing to translate.")
        completion = rag_service.openai.chat.completions.create(
            model=settings.openai_response_model,
            messages=[
                {"role": "system", "content": "Translate technical English into Urdu while preserving code/math."},
                {"role": "user", "content": source_text},
            ],
            temperature=0.1,
        )
        urdu = completion.choices[0].message.content or ""
        return models.TranslateResponse(urdu_text=urdu.strip())
    except Exception as exc:
        raise HTTPException(status_code=500, detail=f"Translation failed: {exc}") from exc


@app.post("/api/personalize", response_model=models.PersonalizeResponse)
def personalize_endpoint(payload: models.PersonalizeRequest):
    prompt = (
        "Create a concise overlay (<=250 words) that complements the chapter excerpt. "
        "Provide:\n"
        "- Beginner insight (plain language)\n"
        "- Advanced insight (deeper math/control theory angle)\n"
        "- Hardware-specific note aligned to the learner preferences\n"
        "Ground everything in the provided excerpt."
    )
    chapter_text = load_chapter(payload.chapter_id)
    excerpt = chapter_text[:1500]
    try:
        completion = rag_service.openai.chat.completions.create(
            model=settings.openai_response_model,
            messages=[
                {"role": "system", "content": prompt},
                {
                    "role": "user",
                    "content": (
                        f"Chapter ID: {payload.chapter_id}\n"
                        f"Desired difficulty: {payload.difficulty}\n"
                        f"Hardware focus: {payload.hardware_focus or 'general'}\n"
                        f"Learning preference: {payload.learning_preference or 'balanced'}\n"
                        f"Software background: {payload.software_background or 'unspecified'}\n"
                        f"Hardware background: {payload.hardware_background or 'unspecified'}\n"
                        f"Learning goal: {payload.learning_goal or 'master humanoid robotics'}\n"
                        f"Excerpt:\n{excerpt}"
                    ),
                },
            ],
            temperature=0.3,
        )
        overlay = completion.choices[0].message.content or ""
        return models.PersonalizeResponse(overlay=overlay.strip())
    except Exception as exc:
        raise HTTPException(status_code=500, detail=f"Personalization failed: {exc}") from exc
