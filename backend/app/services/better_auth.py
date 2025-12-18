from __future__ import annotations

from typing import Optional

from better_auth.api.default_api import DefaultApi
from better_auth.api_client import ApiClient
from better_auth.configuration import Configuration
from better_auth.models import (
    GetSession200Response,
    SignInEmail200Response,
    SignInEmailRequest,
    SignUpWithEmailAndPassword200Response,
    SignUpWithEmailAndPasswordRequest,
)

from backend.app.config import Settings


class BetterAuthService:
    def __init__(self, settings: Settings):
        host = getattr(settings, "better_auth_host", None) or "https://api.better-auth.com"
        configuration = Configuration(host=host)
        if settings.better_auth_secret:
            configuration.access_token = settings.better_auth_secret
        self.api_client = ApiClient(configuration)
        self.api = DefaultApi(self.api_client)

    def signup(self, name: str, email: str, password: str) -> SignUpWithEmailAndPassword200Response:
        request = SignUpWithEmailAndPasswordRequest(name=name, email=email, password=password)
        return self.api.sign_up_with_email_and_password(sign_up_with_email_and_password_request=request)

    def signin(self, email: str, password: str) -> SignInEmail200Response:
        request = SignInEmailRequest(email=email, password=password)
        return self.api.sign_in_email(sign_in_email_request=request)

    def get_session(self, token: str) -> Optional[GetSession200Response]:
        headers = {"Authorization": f"Bearer {token}"}
        return self.api.get_session(_headers=headers)
