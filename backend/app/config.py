from functools import lru_cache
from typing import Optional

from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    model_config = SettingsConfigDict(env_file=".env", env_file_encoding="utf-8", case_sensitive=False)

    openai_api_key: str
    openai_response_model: str = "gpt-4o-mini"
    openai_embedding_model: str = "text-embedding-3-large"

    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection: str

    site_base_url: str = "https://physical-ai-textbook.vercel.app"


@lru_cache
def get_settings() -> Settings:
    return Settings()  # type: ignore[arg-type]
