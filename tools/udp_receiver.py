import socket
import datetime

UDP_IP = "0.0.0.0" # すべてのインターフェースで待ち受け
UDP_PORT = 22222

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"UDP server started on port {UDP_PORT}...")

# 実行した日時のファイル名を自動生成
filename = datetime.datetime.now().strftime("drone_log_%Y%m%d_%H%M%S.txt")
print(f"Saving log to {filename} ...\n")

# ファイルを「追記モード(a)」で開きっぱなしにする
with open(filename, "a", encoding="utf-8") as f:
    while True:
        try:
            data, addr = sock.recvfrom(1024) # バッファサイズを少し余裕持たせました
            text = data.decode('utf-8', errors='ignore')
            
            # 1. コンソール（画面）に表示
            print(text, end="")
            
            # 2. ファイルに書き込んで即座に保存(flush)
            f.write(text)
            f.flush()
            
        except KeyboardInterrupt:
            print("\nExiting...")
            break