from __future__ import annotations

import os
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware

from backend.app import models
from backend.app.config import get_settings
from backend.app.db import get_session, init_db
from backend.app.db_models import UserProfile
from backend.app.services.better_auth import BetterAuthService
from backend.app.services.rag import RAGService
from scripts.ingest_book import ingest_book

settings = get_settings()
app = FastAPI(
    title="Physical AI Backend",
    version="1.0.0",
    description="FastAPI service for textbook QA, ingestion, translation, and personalization.",
)
rag_service = RAGService(settings)
auth_service = BetterAuthService(settings)
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
    except Exception as exc:  # pragma: no cover - ensures friendly errors
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
                {"role": "system", "content": "Translate technical English into Urdu preserving code and math."},
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
        "Create a concise overlay (<=250 words) complementing the base chapter. "
        "Provide:\n"
        "- Beginner insight (plain language)\n"
        "- Advanced insight (deeper math/control theory angle)\n"
        "- Hardware-specific note based on the learner profile\n"
        "Reference the provided chapter excerpt only; do not invent new claims."
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


@app.on_event("startup")
async def on_startup():
    if settings.neon_database_url:
        await init_db()


@app.post("/api/auth/signup", response_model=models.AuthResponse)
async def signup(payload: models.AuthSignupRequest):
    if not settings.neon_database_url:
        raise HTTPException(status_code=500, detail="Database not configured.")
    if not settings.better_auth_secret:
        raise HTTPException(status_code=500, detail="Better-Auth secret missing.")
    try:
        response = auth_service.signup(payload.name, payload.email, payload.password)
        user = response.user
        async with get_session() as session:
            profile = UserProfile(
                user_id=user.id,
                email=user.email,
                software_background=payload.software_background,
                hardware_background=payload.hardware_background,
                learning_preference=payload.learning_preference,
            )
            session.add(profile)
            await session.commit()
        return models.AuthResponse(token=response.token or "", user_id=user.id, email=user.email)
    except Exception as exc:
        raise HTTPException(status_code=400, detail=f"Signup failed: {exc}") from exc


@app.post("/api/auth/signin", response_model=models.AuthResponse)
async def signin(payload: models.AuthSigninRequest):
    if not settings.better_auth_secret:
        raise HTTPException(status_code=500, detail="Better-Auth secret missing.")
    try:
        response = auth_service.signin(payload.email, payload.password)
        user = response.user or {}
        return models.AuthResponse(token=response.token, user_id=_get_user_attr(user, "id", ""), email=payload.email)
    except Exception as exc:
        raise HTTPException(status_code=400, detail=f"Signin failed: {exc}") from exc


@app.get("/api/auth/session")
async def session_info(authorization: str):
    token = authorization.replace("Bearer", "").strip()
    if not token:
        raise HTTPException(status_code=401, detail="Missing token.")
        session = auth_service.get_session(token)
        if not session:
            raise HTTPException(status_code=401, detail="Invalid session.")
    return session.to_dict() if hasattr(session, "to_dict") else {"user": _get_user_attr(session.user, "id")}


@app.post("/api/profile", response_model=models.ProfileUpsertRequest)
async def upsert_profile(payload: models.ProfileUpsertRequest, authorization: str):
    token = authorization.replace("Bearer", "").strip()
    session = auth_service.get_session(token)
    if not session or not session.user:
        raise HTTPException(status_code=401, detail="Invalid session.")
    user_id = _get_user_attr(session.user, "id")
    async with get_session() as db_session:
        profile = await db_session.get(UserProfile, user_id)
        if profile:
            profile.software_background = payload.software_background
            profile.hardware_background = payload.hardware_background
            profile.learning_preference = payload.learning_preference
        else:
            profile = UserProfile(
                user_id=user_id,
                email=_get_user_attr(session.user, "email", ""),
                software_background=payload.software_background,
                hardware_background=payload.hardware_background,
                learning_preference=payload.learning_preference,
            )
            db_session.add(profile)
        await db_session.commit()
    return payload


@app.get("/api/profile", response_model=models.ProfileResponse)
async def get_profile(authorization: str):
    token = authorization.replace("Bearer", "").strip()
    session = auth_service.get_session(token)
    if not session or not session.user:
        raise HTTPException(status_code=401, detail="Invalid session.")
    user_id = _get_user_attr(session.user, "id")
    async with get_session() as db_session:
        profile = await db_session.get(UserProfile, user_id)
        if not profile:
            raise HTTPException(status_code=404, detail="Profile not found.")
        return models.ProfileResponse(
            email=profile.email,
            software_background=profile.software_background,
            hardware_background=profile.hardware_background,
            learning_preference=profile.learning_preference,
        )
def _get_user_attr(user, attr: str, default=None):
    if hasattr(user, attr):
        return getattr(user, attr)
    if isinstance(user, dict):
        return user.get(attr, default)
    return default
