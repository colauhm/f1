import pymysql
import time

class SystemDB:
    def __init__(self):
        # [설정] MySQL 접속 정보
        self.host = '127.0.0.1'
        self.user = 'root'
        self.password = 'qwe123'
        self.db_name = 'motor_log_db'
        self.charset = 'utf8mb4'
        
        self.conn = None
        self.cursor = None
        
        self.init_db()     
        self.connect()     # 처음 한 번만 연결
        self.init_table()  

    def init_db(self):
        try:
            conn = pymysql.connect(
                host=self.host, user=self.user, password=self.password, charset=self.charset
            )
            cursor = conn.cursor()
            cursor.execute(f"CREATE DATABASE IF NOT EXISTS {self.db_name}")
            conn.commit()
            conn.close()
        except Exception as e:
            print(f"[DB Init Error] {e}")

    def connect(self):
        # 연결이 없거나 끊어졌을 때만 재연결
        try:
            if self.conn is None or not self.conn.open:
                self.conn = pymysql.connect(
                    host=self.host,
                    user=self.user,
                    password=self.password,
                    db=self.db_name,
                    charset=self.charset,
                    cursorclass=pymysql.cursors.DictCursor,
                    autocommit=True # [핵심] 자동 커밋 켜기 (속도 향상)
                )
                self.cursor = self.conn.cursor()
                print("[DB Connected] Persistent Connection Opened.")
        except Exception as e:
            print(f"[DB Connection Error] {e}")

    def init_table(self):
        try:
            self.connect() # 연결 확인
            query = """
            CREATE TABLE IF NOT EXISTS driving_logs (
                id INT AUTO_INCREMENT PRIMARY KEY,
                timestamp DOUBLE,
                pedal INT,
                duty FLOAT,
                velocity FLOAT,
                distance FLOAT,
                rpm INT,
                gear INT,
                v_gear VARCHAR(10),
                safety_mode TINYINT,
                warning_msg TEXT,
                restricted TINYINT
            )
            """
            self.cursor.execute(query)
            # 인덱스 추가 (조회 속도용)
            try:
                self.cursor.execute("CREATE INDEX idx_id ON driving_logs(id)")
            except: pass
        except Exception as e:
            print(f"[Table Init Error] {e}")

    # [고속 저장] 연결을 끊지 않고 계속 넣음
    def insert_frame(self, t, p, d, v, dist, rpm, gear, v_gear, safety, msg, r):
        try:
            # 연결 끊겼으면 재접속 시도
            if self.conn is None or not self.conn.open:
                self.connect()

            safety_int = 1 if safety else 0
            
            query = """
            INSERT INTO driving_logs 
            (timestamp, pedal, duty, velocity, distance, rpm, gear, v_gear, safety_mode, warning_msg, restricted) 
            VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)
            """
            self.cursor.execute(query, (t, p, d, v, dist, rpm, gear, v_gear, safety_int, msg, r))
            # autocommit=True라 conn.commit() 생략 가능 (속도 Up)
            
        except Exception as e:
            print(f"[Insert Error] {e}")
            # 에러 나면 다음 턴에 재접속하도록 유도
            try: self.conn.close()
            except: pass
            self.conn = None

    # [조회] 웹소켓용 (읽기 전용 커넥션을 따로 파는 게 안전하지만, 간단히 재사용)
    def fetch_new_logs(self, last_id):
        try:
            if self.conn is None or not self.conn.open:
                self.connect()
                
            query = "SELECT * FROM driving_logs WHERE id > %s ORDER BY id ASC"
            self.cursor.execute(query, (last_id,))
            rows = self.cursor.fetchall()
            
            result = []
            max_id = last_id
            
            for r in rows:
                if r['id'] > max_id:
                    max_id = r['id']
                    
                result.append({
                    "id": r['id'],
                    "t": r['timestamp'] * 1000,
                    "p": r['pedal'],
                    "d": r['duty'],
                    "v": r['velocity'],
                    "dist": r['distance'],
                    "rpm": r['rpm'],
                    "gear": r['gear'],
                    "v_gear_char": r['v_gear'],
                    "safety_mode": bool(r['safety_mode']),
                    "reason": r['warning_msg'],
                    "r": r['restricted']
                })
            return result, max_id
        except Exception as e:
            print(f"[Fetch Error] {e}")
            return [], last_id
            
    def close(self):
        if self.conn:
            self.conn.close()