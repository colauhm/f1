import os
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from contextlib import asynccontextmanager

# router 폴더 내의 webSocket 모듈 임포트
from .routers import webSocket

# ---- 경로 설정 (핵심 수정 부분) ----
# 1. 현재 파일(main.py)의 절대 경로를 구함
current_file_path = os.path.abspath(__file__)
# 2. server 폴더 경로 구함
server_dir = os.path.dirname(current_file_path)
# 3. 상위 폴더(project_folder) 경로 구함
root_dir = os.path.dirname(server_dir)
# 4. app 폴더의 최종 절대 경로 완성
APP_DIR = os.path.join(root_dir, "app")

# [디버깅용] 터미널에 이 경로가 출력되는지 확인!
print(f"---------------------------------------------------")
print(f"✅ App Directory Path: {APP_DIR}")
if os.path.exists(os.path.join(APP_DIR, "style.css")):
    print(f"✅ style.css found OK!")
else:
    print(f"❌ ERROR: style.css NOT found at {APP_DIR}")
print(f"---------------------------------------------------")

# ---- 수명 주기 설정 ----
@asynccontextmanager
async def lifespan(app: FastAPI):
    print("Hardware Threads Starting...")
    webSocket.start_hardware()
    yield
    print("Server Shutting Down...")

app = FastAPI(lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=['*'], 
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(webSocket.router)

# ---- 정적 파일 연결 ----
# /static 경로로 들어오면 APP_DIR 폴더의 파일을 보여줌
app.mount("/static", StaticFiles(directory=APP_DIR), name="static")

# ---- 루트 접속 시 index.html 반환 ----
@app.get("/")
async def read_root():
    # index.html 파일 경로도 절대 경로로 지정
    index_file = os.path.join(APP_DIR, "index.html")
    return FileResponse(index_file)