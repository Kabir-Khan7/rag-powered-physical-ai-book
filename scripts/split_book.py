import re
import textwrap
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BOOK_PATH = ROOT / "Book" / "book.md"
DOCS_DIR = ROOT / "website" / "docs" / "chapters"

STOPWORDS = {
    "the",
    "and",
    "for",
    "with",
    "that",
    "from",
    "this",
    "into",
    "your",
    "about",
    "have",
    "will",
    "their",
    "into",
    "through",
    "when",
    "they",
    "more",
    "than",
    "into",
    "what",
    "while",
    "over",
    "such",
    "these",
    "been",
    "were",
}


def load_book() -> str:
    if not BOOK_PATH.exists():
        raise FileNotFoundError(f"Missing book file at {BOOK_PATH}")
    return BOOK_PATH.read_text(encoding="utf-8")


def extract_chapters(raw: str):
    pattern = re.compile(r"^(#\s+Chapter\s+\d+:[^\n]*)", re.MULTILINE)
    matches = list(pattern.finditer(raw))
    if not matches:
        raise ValueError("No chapters found; ensure headings start with '# Chapter N:'")

    chapters = []
    for idx, match in enumerate(matches):
        start = match.start()
        end = matches[idx + 1].start() if idx + 1 < len(matches) else len(raw)
        block = raw[start:end].strip()
        header_line = match.group(1).strip()
        title = header_line.lstrip("#").strip()
        number = re.search(r"Chapter\s+(\d+)", title)
        chapter_id = f"chapter-{number.group(1) if number else idx + 1}"
        chapters.append(
            {
                "id": chapter_id,
                "title": title,
                "body": block,
            }
        )
    return chapters


def summarize(body: str, max_sentences: int = 3) -> str:
    stripped = re.sub(r"^#[^\n]*\n?", "", body, count=1).strip()
    text = stripped if stripped else body
    text = re.sub(r"`{1,3}.*?`{1,3}", "", text, flags=re.DOTALL)
    sentences = re.split(r"(?<=[.!?])\s+", text)
    cleaned = [s.strip() for s in sentences if s.strip()]
    snippet = " ".join(cleaned[:max_sentences]).strip()
    return snippet[:500]


def infer_difficulty(title: str, body: str) -> str:
    title_lower = title.lower()
    body_lower = body.lower()
    if "introduction" in title_lower or "overview" in title_lower:
        return "beginner"
    if "advanced" in title_lower:
        return "advanced"
    if any(keyword in body_lower for keyword in ("derivation", "control theory", "optimization")):
        return "advanced"
    if any(keyword in body_lower for keyword in ("hands-on", "simulation", "setup", "implementation")):
        return "intermediate"
    return "intermediate"


def extract_keywords(body: str, limit: int = 6):
    words = re.findall(r"[A-Za-z][A-Za-z\-]+", body.lower())
    filtered = [w for w in words if w not in STOPWORDS and len(w) > 4]
    counts = Counter(filtered)
    keywords = [word for word, _ in counts.most_common(limit)]
    return keywords


def ensure_docs_dir():
    DOCS_DIR.mkdir(parents=True, exist_ok=True)


def write_chapter(chapter: dict):
    ensure_docs_dir()
    summary = summarize(chapter["body"])
    difficulty = infer_difficulty(chapter["title"], chapter["body"])
    keywords = extract_keywords(chapter["body"])
    if not keywords:
        keywords = ["robotics"]

    wrapped_summary = textwrap.fill(summary, width=98)
    indented_summary = wrapped_summary.replace("\n", "\n  ")

    frontmatter = [
        "---",
        f"id: {chapter['id']}",
        f"title: \"{chapter['title']}\"",
        f"summary: >-\n  {indented_summary}",
        f"difficulty: {difficulty}",
        f"keywords:",
    ]
    if chapter.get("order", 0) == 0:
        frontmatter.insert(2, "slug: /")
    frontmatter.extend([f"  - {kw}" for kw in keywords])
    frontmatter.append("---\n")

    content = "\n".join(frontmatter) + chapter["body"] + "\n"
    outfile = DOCS_DIR / f"{chapter['id']}.md"
    outfile.write_text(content, encoding="utf-8")
    print(f"Wrote {outfile.relative_to(ROOT)}")


def main():
    raw = load_book()
    chapters = extract_chapters(raw)
    for idx, chapter in enumerate(chapters):
        chapter["order"] = idx
        write_chapter(chapter)


if __name__ == "__main__":
    main()
