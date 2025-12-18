"""
Chunk chapter markdown files, generate OpenAI embeddings, and upsert them into Qdrant.

Environment variables:
  OPENAI_API_KEY          - required
  OPENAI_EMBEDDING_MODEL  - optional (default: text-embedding-3-large)
  QDRANT_URL              - required, e.g. https://xxxxxx.us-east-1-0.aws.cloud.qdrant.io
  QDRANT_API_KEY          - required
  QDRANT_COLLECTION       - required collection name
  SITE_BASE_URL           - optional, used for source_url metadata (default Vercel URL)
"""

from __future__ import annotations

import os
import re
import sys
import textwrap
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.http import models as qmodels
from dotenv import load_dotenv

ROOT = Path(__file__).resolve().parents[1]
DOCS_DIR = ROOT / "website" / "docs" / "chapters"
DEFAULT_MODEL = "text-embedding-3-large"
DEFAULT_SITE = "https://physical-ai-textbook.vercel.app"
TARGET_TOKENS = 800
OVERLAP_TOKENS = 150


def load_tokenizer():
    try:
        import tiktoken

        return tiktoken.get_encoding("cl100k_base")
    except Exception:
        return None


TOKENIZER = load_tokenizer()


def tokenize(text: str) -> List[int]:
    if TOKENIZER:
        return TOKENIZER.encode(text)
    # Approximate tokens by splitting on whitespace and punctuation
    cleaned = re.findall(r"\w+|\S", text)
    return list(range(len(cleaned)))


def detokenize(tokens: Sequence[int], reference: str) -> str:
    if TOKENIZER:
        return TOKENIZER.decode(tokens)
    words = re.findall(r"\w+|\S", reference)
    selected = []
    for idx in tokens:
        if 0 <= idx < len(words):
            selected.append(words[idx])
    return " ".join(selected)


def chunk_text(text: str, max_tokens: int = TARGET_TOKENS, overlap: int = OVERLAP_TOKENS) -> List[str]:
    tokens = tokenize(text)
    chunks: List[str] = []
    start = 0
    total = len(tokens)
    while start < total:
        end = min(start + max_tokens, total)
        if TOKENIZER:
            chunk = TOKENIZER.decode(tokens[start:end])
        else:
            # fallback uses substring slicing on original text via line boundaries
            approx_chars = int((end - start) * 4)
            chunk = text[start * 4 : start * 4 + approx_chars]
        chunks.append(chunk.strip())
        if end == total:
            break
        start = max(0, end - overlap)
    return [c for c in chunks if c]


def read_chapter(path: Path) -> Tuple[str, str]:
    raw = path.read_text(encoding="utf-8")
    body = raw
    if raw.startswith("---"):
        try:
            _, _, remainder = raw.split("---", 2)
            body = remainder.strip()
        except ValueError:
            body = raw
    return path.stem, body.strip()


def extract_sections(body: str) -> List[Tuple[str, str]]:
    lines = body.splitlines()
    sections: List[Tuple[str, List[str]]] = []
    current_heading = ""
    buffer: List[str] = []
    heading_pattern = re.compile(r"^(#{1,6})\s+(.*)")

    def flush():
        nonlocal buffer, current_heading
        if buffer:
            sections.append((current_heading or "Overview", buffer))
            buffer = []

    for line in lines:
        match = heading_pattern.match(line)
        if match:
            flush()
            current_heading = match.group(2).strip()
        else:
            buffer.append(line)
    flush()

    flattened: List[Tuple[str, str]] = []
    for heading, paragraphs in sections:
        text = "\n".join(paragraphs).strip()
        if text:
            flattened.append((heading, text))
    return flattened


@dataclass
class Chunk:
    chapter_id: str
    heading: str
    text: str
    order: int

    @property
    def point_id(self) -> str:
        safe_heading = re.sub(r"[^a-z0-9]+", "-", self.heading.lower()).strip("-")
        return f"{self.chapter_id}-{self.order}-{safe_heading}"[:64]

    @property
    def snippet(self) -> str:
        return textwrap.shorten(self.text.replace("\n", " "), width=280, placeholder="…")

    @property
    def source_url(self) -> str:
        base = os.getenv("SITE_BASE_URL", DEFAULT_SITE).rstrip("/")
        slug = "" if self.chapter_id == "chapter-1" else f"/chapters/{self.chapter_id}"
        return f"{base}{slug or '/'}#chunk-{self.order}"


def build_chunks() -> List[Chunk]:
    if not DOCS_DIR.exists():
        raise FileNotFoundError(f"Docs directory missing at {DOCS_DIR}")
    chunks: List[Chunk] = []
    for path in sorted(DOCS_DIR.glob("chapter-*.md")):
        chapter_id, body = read_chapter(path)
        sections = extract_sections(body)
        seq = 0
        for heading, text in sections:
            for piece in chunk_text(text):
                chunks.append(Chunk(chapter_id, heading, piece, seq))
                seq += 1
    if not chunks:
        raise RuntimeError("No chunks generated; ensure chapter files exist.")
    return chunks


def ensure_collection(client: QdrantClient, collection: str, vector_size: int):
    existing = client.get_collections()
    names = {c.name for c in existing.collections or []}
    if collection in names:
        return
    client.create_collection(
        collection_name=collection,
        vectors_config=qmodels.VectorParams(size=vector_size, distance=qmodels.Distance.COSINE),
    )


def ingest_book() -> int:
    openai_key = os.getenv("OPENAI_API_KEY")
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_key = os.getenv("QDRANT_API_KEY")
    collection = os.getenv("QDRANT_COLLECTION")
    if not all([openai_key, qdrant_url, qdrant_key, collection]):
        missing = [name for name, value in [
            ("OPENAI_API_KEY", openai_key),
            ("QDRANT_URL", qdrant_url),
            ("QDRANT_API_KEY", qdrant_key),
            ("QDRANT_COLLECTION", collection),
        ] if not value]
        print(f"Missing required environment variables: {', '.join(missing)}", file=sys.stderr)
        sys.exit(1)

    chunks = build_chunks()
    client = OpenAI(api_key=openai_key)
    qdrant = QdrantClient(url=qdrant_url, api_key=qdrant_key, timeout=30.0)
    model = os.getenv("OPENAI_EMBEDDING_MODEL", DEFAULT_MODEL)

    preview_embedding = client.embeddings.create(model=model, input="Physical AI")
    vector_size = len(preview_embedding.data[0].embedding)
    ensure_collection(qdrant, collection, vector_size)

    batch_points = []
    for chunk in chunks:
        embedding = client.embeddings.create(model=model, input=chunk.text).data[0].embedding
        payload = {
            "chapter_id": chunk.chapter_id,
            "heading": chunk.heading,
            "source_url": chunk.source_url,
            "snippet": chunk.snippet,
            "order": chunk.order,
        }
        point = qmodels.PointStruct(id=chunk.point_id, vector=embedding, payload=payload)
        batch_points.append(point)

        if len(batch_points) >= 32:
            qdrant.upsert(collection_name=collection, points=batch_points)
            batch_points = []

    if batch_points:
        qdrant.upsert(collection_name=collection, points=batch_points)

    return len(chunks)


def main():
    count = ingest_book()
    print(f"Ingested {count} chunks into collection '{os.environ.get('QDRANT_COLLECTION')}'.")


if __name__ == "__main__":
    main()
