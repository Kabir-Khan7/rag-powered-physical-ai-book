from __future__ import annotations

from typing import List

from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.http.models import ScoredPoint

from backend.app.config import Settings
from backend.app.models import Citation, QARequest, QAResponse


class RAGService:
    def __init__(self, settings: Settings):
        self.settings = settings
        self.openai = OpenAI(api_key=settings.openai_api_key)
        self.qdrant = QdrantClient(url=settings.qdrant_url, api_key=settings.qdrant_api_key, timeout=30.0)

    def _embed(self, text: str) -> List[float]:
        response = self.openai.embeddings.create(
            model=self.settings.openai_embedding_model,
            input=text,
        )
        return response.data[0].embedding

    def _search(self, query: str, limit: int) -> List[ScoredPoint]:
        vector = self._embed(query)
        results = self.qdrant.search(
            collection_name=self.settings.qdrant_collection,
            query_vector=vector,
            limit=limit,
            with_payload=True,
        )
        return results

    def _build_prompt(self, request: QARequest, contexts: List[Citation]) -> str:
        formatted_contexts = "\n\n".join(
            f"[{idx + 1}] {c.heading} — {c.snippet}" for idx, c in enumerate(contexts)
        )
        instructions = (
            "You are a Physical AI textbook assistant. Answer using only the provided context. "
            "If the answer is not contained, reply that the book does not cover it yet. "
            "Always cite supporting IDs in square brackets like [1]."
        )
        return (
            f"{instructions}\n\n"
            f"# Context\n{formatted_contexts}\n\n"
            f"# Question\n{request.query}"
        )

    def _call_llm(self, prompt: str) -> str:
        completion = self.openai.chat.completions.create(
            model=self.settings.openai_response_model,
            messages=[
                {"role": "system", "content": "You are a Physical AI tutor. Use only provided context."},
                {"role": "user", "content": prompt},
            ],
            temperature=0.2,
        )
        return completion.choices[0].message.content or ""

    def answer(self, request: QARequest) -> QAResponse:
        if request.selected_text:
            citation = Citation(
                heading="User Selected Passage",
                source_url=self.settings.site_base_url,
                snippet=request.selected_text[:250],
            )
            prompt = (
                "Use only the user-selected text to answer. "
                "If the information is missing, state that explicitly.\n\n"
                f"Selected text:\n{request.selected_text}\n\nQuestion:\n{request.query}"
            )
            answer = self._call_llm(prompt)
            return QAResponse(answer=answer.strip(), citations=[citation])

        search_results = self._search(request.query, request.top_k)
        if not search_results:
            return QAResponse(answer="I could not find relevant passages in the textbook.", citations=[])

        citations = []
        for point in search_results:
            payload = point.payload or {}
            citations.append(
                Citation(
                    heading=payload.get("heading", "Chapter"),
                    source_url=payload.get("source_url") or self.settings.site_base_url,
                    snippet=(payload.get("snippet") or "")[:280],
                )
            )

        prompt = self._build_prompt(request, citations)
        answer = self._call_llm(prompt)
        return QAResponse(answer=answer.strip(), citations=citations)
