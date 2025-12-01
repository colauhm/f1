import os

from fastapi import FastAPI

from fastapi.middleware.cors import CORSMiddleware

from fastapi.staticfiles import StaticFiles

from fastapi.responses import FileResponse

from contextlib import asynccontextmanager



# 작성한 webSocket 모듈 임포트

from .routers import pedalLogic



# ---- 경로 설정 ----

current_file_path = os.path.abspath(__file__)

server_dir = os.path.dirname(current_file_path)

root_dir = os.path.dirname(server_dir)

APP_DIR = os.path.join(root_dir, "app")



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

    # 여기서 모터 제어 스레드를 실행합니다!

    pedalLogic.start_hardware()

    yield

    print("Server Shutting Down...")

    pedalLogic.stop_threads = True # 스레드 종료 신호



app = FastAPI(lifespan=lifespan)



# server/main.py



app.add_middleware(

    CORSMiddleware,

    allow_origins=["*"],      # 모든 IP 허용

    allow_credentials=False,  # [★핵심 수정] True -> False로 변경

    allow_methods=["*"],

    allow_headers=["*"],

)



# 라우터 등록 ( /ws 경로 )

app.include_router(pedalLogic.router)



# 정적 파일 및 루트 경로

app.mount("/static", StaticFiles(directory=APP_DIR), name="static")



@app.get("/")

async def read_root():

    index_file = os.path.join(APP_DIR, "index.html")

    return FileResponse(index_file)