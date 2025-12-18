from __future__ import annotations

from contextlib import asynccontextmanager
from typing import AsyncIterator

from sqlalchemy.ext.asyncio import AsyncSession, async_sessionmaker, create_async_engine
from sqlalchemy.orm import DeclarativeBase

from backend.app.config import get_settings


class Base(DeclarativeBase):
    pass


settings = get_settings()
engine = (
    create_async_engine(settings.neon_database_url, echo=False)
    if settings.neon_database_url
    else None
)
session_factory = async_sessionmaker(engine, expire_on_commit=False) if engine else None


@asynccontextmanager
async def get_session() -> AsyncIterator[AsyncSession]:
    if session_factory is None:
        raise RuntimeError("Database session not configured. Set NEON_DATABASE_URL.")
    session = session_factory()
    try:
        yield session
    finally:
        await session.close()


async def init_db():
    if engine is None:
        raise RuntimeError("Database engine not configured. Set NEON_DATABASE_URL.")
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)
