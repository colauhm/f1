from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .routers import webSocket
#from .database import fileload
from starlette.middleware.trustedhost import TrustedHostMiddleware
app = FastAPI()
app.add_middleware(TrustedHostMiddleware, allowed_hosts=["*"])
app.add_middleware(
    CORSMiddleware,
		allow_origins='*',  # 모든 오리진(도메인)에서의 요청을 허용합니다. 실제 운영 환경에서는 '*' 대신 특정 도메인을 지정하는 것이 보안상 좋습니다.
    allow_credentials=True,  # 요청에 인증 정보를 포함할 수 있도록 허용합니다. 예를 들어, 쿠키 기반의 인증을 사용하는 경우 필요합니다.
    allow_methods=["*"],  # 모든 HTTP 메서드를 허용합니다. 실제로 필요한 메서드만 지정하는 것이 좋습니다.
    allow_headers=["*"],  # 모든 HTTP 헤더를 허용합니다. 필요한 경우 원하는 헤더만 지정할 수 있습니다.
)


app.include_router(webSocket.router)