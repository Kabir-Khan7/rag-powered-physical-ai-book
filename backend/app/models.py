from typing import List, Optional

from pydantic import BaseModel, EmailStr, Field, HttpUrl, model_validator


class QARequest(BaseModel):
    query: str = Field(..., min_length=3)
    selected_text: Optional[str] = Field(default=None, description="Optional user-selected passage to constrain answers.")
    top_k: int = Field(default=5, ge=1, le=12)


class Citation(BaseModel):
    heading: str
    source_url: HttpUrl
    snippet: str


class QAResponse(BaseModel):
    answer: str
    citations: List[Citation]


class IngestResponse(BaseModel):
    chunks_ingested: int
    collection: str


class TranslateRequest(BaseModel):
    text: Optional[str] = Field(default=None, description="English text to translate.")
    chapter_id: Optional[str] = None

    @model_validator(mode="after")
    def check_payload(cls, values):
        text = values.text
        chapter_id = values.chapter_id
        if not text and not chapter_id:
            raise ValueError("Provide raw text or a chapter_id.")
        return values


class TranslateResponse(BaseModel):
    urdu_text: str


class PersonalizeRequest(BaseModel):
    chapter_id: str
    difficulty: str = Field(default="beginner")
    hardware_focus: Optional[str] = None
    learning_preference: Optional[str] = Field(
        default=None, description="e.g., visual, hands-on, theoretical"
    )


class PersonalizeResponse(BaseModel):
    overlay: str


class AuthSignupRequest(BaseModel):
    name: str
    email: EmailStr
    password: str = Field(..., min_length=8)
    software_background: str
    hardware_background: str
    learning_preference: str


class AuthResponse(BaseModel):
    token: str
    user_id: str
    email: EmailStr


class AuthSigninRequest(BaseModel):
    email: EmailStr
    password: str = Field(..., min_length=8)


class ProfileUpsertRequest(BaseModel):
    software_background: str
    hardware_background: str
    learning_preference: str


class ProfileResponse(ProfileUpsertRequest):
    email: EmailStr
