import socket

UDP_IP = "0.0.0.0" # すべてのインターフェースで待ち受け
UDP_PORT = 22222

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"UDP server started on port {UDP_PORT}...")

while True:
    data, addr = sock.recvfrom(256) # バッファサイズ 256バイト
    print(f"{data.decode('utf-8', errors='ignore')}", end="")