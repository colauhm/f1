# f1

## 라이브러리 설치

python3 -m venv myenv

source myenv/bin/activate

pip install -r backUp.txt

## 서버 실행

 uvicorn server.main:app --host 0.0.0.0 --port 8000 --reload