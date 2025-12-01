import os
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from contextlib import asynccontextmanager

# [중요] 폴더 구조가 server/routers/pedalLogic.py 인 경우:
# main.py가 server 폴더 안에 있다면:
from routers import pedalLogic 
# 만약 에러가 나면 "from .routers import pedalLogic" 시도

# ---- 경로 설정 ----
current_file_path = os.path.abspath(__file__)
server_dir = os.path.dirname(current_file_path)
# 상위 폴더(루트) 찾기
root_dir = os.path.dirname(server_dir) 
# app 폴더 경로 (HTML, CSS 있는 곳)
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
    pedalLogic.start_hardware()  # 하드웨어 시작
    yield
    print("Server Shutting Down...")
    pedalLogic.stop_threads = True # 스레드 종료

app = FastAPI(lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],      
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 라우터 등록
app.include_router(pedalLogic.router)

# 정적 파일 마운트 (CSS, JS 등을 위함)
app.mount("/static", StaticFiles(directory=APP_DIR), name="static")

# 루트 경로 (index.html 서빙)
@app.get("/")
async def read_root():
    index_file = os.path.join(APP_DIR, "index.html")
    return FileResponse(index_file)

if __name__ == "__main__":
    import uvicorn
    # main:app 이라고 써야 리로드 시 문제없음. 아니면 그냥 app
    uvicorn.run(app, host="0.0.0.0", port=8000)