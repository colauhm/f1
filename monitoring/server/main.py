import os
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from contextlib import asynccontextmanager

# [중요] 라우터 임포트
# 폴더 구조상 server 패키지 내부에서의 상대 경로 임포트 사용
from .routers import webSocket

# ---- 수명 주기(Lifespan) 설정 ----
# 서버가 켜질 때 스레드를 시작하고, 꺼질 때 정리하는 로직
@asynccontextmanager
async def lifespan(app: FastAPI):
    # 시작 시 실행: 모터 스레드 켜기
    print("Hardware Threads Starting...")
    webSocket.start_hardware()
    yield
    # 종료 시 실행 (필요 시 추가)
    print("Server Shutting Down...")

# 앱 생성 (lifespan 적용)
app = FastAPI(lifespan=lifespan)

# 미들웨어 설정
app.add_middleware(
    CORSMiddleware,
    allow_origins=['*'], 
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 라우터 등록
app.include_router(webSocket.router)

# ---- 정적 파일 및 프론트엔드 연결 ----
# 현재 파일(main.py) 위치를 기준으로 ../app 폴더 경로 계산
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
APP_DIR = os.path.join(BASE_DIR, "..", "app")

# 1. /static 경로로 app 폴더 내의 css, js 파일 제공
app.mount("/static", StaticFiles(directory=APP_DIR), name="static")

# 2. 루트(/) 접속 시 index.html 반환
@app.get("/")
async def read_root():
    return FileResponse(os.path.join(APP_DIR, "index.html"))