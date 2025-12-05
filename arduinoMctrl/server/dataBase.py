import pymysql
import time

class SystemDB:
    def __init__(self):
        # [설정] 여기에 너의 MySQL 접속 정보를 적어야 한다.
        self.host = '127.0.0.1'   # 라즈베리 파이 로컬에 깔렸으면 127.0.0.1, 다른 PC면 그 IP
        self.user = 'root'        # MySQL 아이디 (보통 root)
        self.password = 'qwe123'    # MySQL 비밀번호
        self.db_name = 'motor_log_db' # 사용할 데이터베이스 이름
        self.charset = 'utf8mb4'
        
        self.init_db()     # DB 생성 (없으면)
        self.init_table()  # 테이블 생성

    def _get_conn(self):
        # MySQL 서버에 연결
        return pymysql.connect(
            host=self.host,
            user=self.user,
            password=self.password,
            db=self.db_name,
            charset=self.charset,
            cursorclass=pymysql.cursors.DictCursor # 데이터를 딕셔너리로 가져오기 위함
        )

    def init_db(self):
        # DB가 아예 없을 수도 있으니 접속해서 생성 시도
        conn = pymysql.connect(
            host=self.host, user=self.user, password=self.password, charset=self.charset
        )
        cursor = conn.cursor()
        cursor.execute(f"CREATE DATABASE IF NOT EXISTS {self.db_name}")
        conn.commit()
        conn.close()

    def init_table(self):
        conn = self._get_conn()
        cursor = conn.cursor()
        
        # MySQL 문법에 맞게 수정됨 (AUTOINCREMENT -> AUTO_INCREMENT, REAL -> FLOAT 등)
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
        cursor.execute(query)
        conn.commit()
        conn.close()

    # [저장]
    def insert_frame(self, t, p, d, v, dist, rpm, gear, v_gear, safety, msg, r):
        conn = self._get_conn()
        cursor = conn.cursor()
        
        safety_int = 1 if safety else 0
        
        # MySQL은 ? 대신 %s 를 사용한다.
        query = """
        INSERT INTO driving_logs 
        (timestamp, pedal, duty, velocity, distance, rpm, gear, v_gear, safety_mode, warning_msg, restricted) 
        VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)
        """
        cursor.execute(query, (t, p, d, v, dist, rpm, gear, v_gear, safety_int, msg, r))
        conn.commit()
        conn.close()

    # [조회]
    def fetch_new_logs(self, last_id):
        conn = self._get_conn()
        cursor = conn.cursor()
        
        query = "SELECT * FROM driving_logs WHERE id > %s ORDER BY id ASC"
        cursor.execute(query, (last_id,))
        rows = cursor.fetchall() # DictCursor 덕분에 딕셔너리 리스트로 나옴
        conn.close()
        
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