import socket
import tkinter as tk

# Ağ Ayarları
UDP_IP = "192.168.4.1"
UDP_PORT = 8080

# UDP Soketini Başlat (Hem dinleme hem gönderme için)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT)) # Tüm ağ arayüzlerinden gelen 8080 portunu dinle
sock.setblocking(False)          # Arayüzün donmaması için non-blocking mod

def send_cmd(cmd):
    sock.sendto(cmd.encode(), (UDP_IP, UDP_PORT))
    print(f"Komut Fırlatıldı: {cmd}")

# --- TELEMETRİ DİNLEYİCİSİ ---
def listen_telemetry():
    try:
        while True:
            # Gelen veriyi oku (Non-blocking olduğu için veri yoksa hata fırlatır, except'e düşer)
            data, addr = sock.recvfrom(1024)
            text = data.decode('utf-8').strip()
            
            # Gelen paket bizim telemetri formatımıza uyuyor mu?
            if text.startswith("START") and text.endswith("END"):
                parts = text.split(',')
                if len(parts) >= 7:
                    r_angle = float(parts[1])
                    t_angle = float(parts[2])
                    accel   = float(parts[3])
                    speed   = float(parts[4])
                    state   = int(parts[5])
                    
                    state_str = "INIT" if state == 0 else "BALANCING" if state == 1 else "FAULT"
                    
                    # Arayüzdeki yazıları güncelle
                    lbl_val_angle.config(text=f"{r_angle:.2f}°")
                    lbl_val_target.config(text=f"{t_angle:.2f}°")
                    lbl_val_speed.config(text=f"{speed:.1f}")
                    lbl_val_state.config(text=state_str, fg="#2ecc71" if state == 1 else "#e74c3c")
                    
    except BlockingIOError:
        pass # Şu an tamponda okunacak yeni paket yok, devam eder
    except Exception as e:
        pass
        
    # Bu fonksiyonu 50 milisaniye sonra tekrar çalıştır (Sürekli döngü)
    root.after(50, listen_telemetry)

#  SÜRÜŞ KOMUTLARI (Spam Korumalı ve Focus Kontrollü)
keys_pressed = {'w': False, 's': False, 'a': False, 'd': False}

# Eğer PID kutusuna (entry_cmd) yazı yazılıyorsa, sürüş tuşlarını görmezden gel
def is_typing_pid():
    return root.focus_get() == entry_cmd

def press_w(event):
    if is_typing_pid(): return
    if not keys_pressed['w']:
        keys_pressed['w'] = True
        send_cmd("Y 150")

def release_w(event):
    if is_typing_pid(): return
    if keys_pressed['w']:
        keys_pressed['w'] = False
        send_cmd("Y 0")

def press_s(event):
    if is_typing_pid(): return
    if not keys_pressed['s']:
        keys_pressed['s'] = True
        send_cmd("Y -150")

def release_s(event):
    if is_typing_pid(): return
    if keys_pressed['s']:
        keys_pressed['s'] = False
        send_cmd("Y 0")

def press_a(event):
    if is_typing_pid(): return
    if not keys_pressed['a']:
        keys_pressed['a'] = True
        send_cmd("X -150")

def release_a(event):
    if is_typing_pid(): return
    if keys_pressed['a']:
        keys_pressed['a'] = False
        send_cmd("X 0")

def press_d(event):
    if is_typing_pid(): return
    if not keys_pressed['d']:
        keys_pressed['d'] = True
        send_cmd("X 150")

def release_d(event):
    if is_typing_pid(): return
    if keys_pressed['d']:
        keys_pressed['d'] = False
        send_cmd("X 0")

# --- PID KOMUT GÖNDERME ---
def send_custom_cmd(event=None):
    cmd = entry_cmd.get().strip()
    if cmd:
        send_cmd(cmd)
        entry_cmd.delete(0, tk.END)
        root.focus_set() 

#  ARAYÜZ 
root = tk.Tk()
root.title("Denge Robotu Kontrol ve Telemetri")
root.geometry("500x450")
root.configure(bg="#2c3e50")

# TELEMETRİ PANELİ 
frame_telemetry = tk.LabelFrame(root, text=" CANLI TELEMETRİ ", font=("Arial", 10, "bold"), fg="#f1c40f", bg="#34495e", padx=10, pady=10)
frame_telemetry.pack(fill="x", padx=20, pady=15)

tk.Label(frame_telemetry, text="Robot Açısı:", font=("Arial", 11), bg="#34495e", fg="white").grid(row=0, column=0, sticky="e", padx=5)
lbl_val_angle = tk.Label(frame_telemetry, text="0.00°", font=("Arial", 11, "bold"), bg="#34495e", fg="#3498db")
lbl_val_angle.grid(row=0, column=1, sticky="w")

tk.Label(frame_telemetry, text="Hedef Açı:", font=("Arial", 11), bg="#34495e", fg="white").grid(row=0, column=2, sticky="e", padx=5, pady=5)
lbl_val_target = tk.Label(frame_telemetry, text="0.00°", font=("Arial", 11, "bold"), bg="#34495e", fg="#e67e22")
lbl_val_target.grid(row=0, column=3, sticky="w")

tk.Label(frame_telemetry, text="Motor Hızı:", font=("Arial", 11), bg="#34495e", fg="white").grid(row=1, column=0, sticky="e", padx=5, pady=5)
lbl_val_speed = tk.Label(frame_telemetry, text="0.0", font=("Arial", 11, "bold"), bg="#34495e", fg="#9b59b6")
lbl_val_speed.grid(row=1, column=1, sticky="w")

tk.Label(frame_telemetry, text="Sistem Durumu:", font=("Arial", 11), bg="#34495e", fg="white").grid(row=1, column=2, sticky="e", padx=5)
lbl_val_state = tk.Label(frame_telemetry, text="BEKLENIYOR", font=("Arial", 11, "bold"), bg="#34495e", fg="#95a5a6")
lbl_val_state.grid(row=1, column=3, sticky="w")

#  SÜRÜŞ BİLGİSİ 
label_drive = tk.Label(root, text="SÜRÜŞ: W, A, S, D Tuşlarını Kullanın\n(Bu pencere seçiliyken geçerlidir)", 
                 font=("Arial", 11, "bold"), fg="#ecf0f1", bg="#2c3e50")
label_drive.pack(pady=10)

root.bind('<KeyPress-w>', press_w)
root.bind('<KeyRelease-w>', release_w)
root.bind('<KeyPress-s>', press_s)
root.bind('<KeyRelease-s>', release_s)
root.bind('<KeyPress-a>', press_a)
root.bind('<KeyRelease-a>', release_a)
root.bind('<KeyPress-d>', press_d)
root.bind('<KeyRelease-d>', release_d)

# PID İNPUT PANELİ 
label_pid = tk.Label(root, text="PID ve Diğer Komutlar: (Örn: P 3.5, I 0.1, R 0)", font=("Arial", 10), fg="#bdc3c7", bg="#2c3e50")
label_pid.pack(pady=5)

entry_cmd = tk.Entry(root, font=("Arial", 14), justify="center", width=15)
entry_cmd.pack(pady=5)
entry_cmd.bind('<Return>', send_custom_cmd)

# Ana pencereye odaklan ve dinleyiciyi başlat
root.focus_set()
root.after(50, listen_telemetry) # Telemetri döngüsünü tetikle
root.mainloop()