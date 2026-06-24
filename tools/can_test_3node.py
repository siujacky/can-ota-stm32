#!/usr/bin/env python3
"""
3-Node CAN Bus Test: Pico2(XL2515) ↔ MCP2515 ↔ Blue Pill(bxCAN)
Tests all TX/RX paths with proper BKP clearing.

Root cause fixed: BKP_DR1=0xB001 caused 30s bootloader window.
All tests now clear BKP before reset so bootloader uses 500ms window.
Echo: bxCAN returns id+1 frame when it receives any frame (proves RX).
"""
import serial, time, subprocess, struct, threading, sys

PORT = '/dev/ttyACM1'
BCG  = '/tmp/claude-0/-home-pi-ros2/3120f764-2cff-4bcd-8711-a3cbcdab5e5c/scratchpad/bcg'

def rram(addr, sz=4):
    r = subprocess.run(['st-flash','read','/tmp/r.bin',f'0x{addr:08X}',str(sz)], capture_output=True)
    d = open('/tmp/r.bin','rb').read()[:sz]
    return d if len(d) == sz else None

def clear_bkp():
    """Clear BKP_DR1 so bootloader uses 500ms window (not 30s)."""
    # Enable RCC APB1: PWREN(bit28) + BKPEN(bit27)
    r = rram(0x4002101C)
    if r:
        v = struct.unpack('<I',r)[0] | (3<<27)
        open('/tmp/wb.bin','wb').write(struct.pack('<I',v))
        subprocess.run(['st-flash','write','/tmp/wb.bin','0x4002101C'], capture_output=True)
    time.sleep(0.05)
    # Enable DBP in PWR_CR (bit8)
    r = rram(0x40007000)
    if r:
        v = struct.unpack('<I',r)[0] | (1<<8)
        open('/tmp/wb.bin','wb').write(struct.pack('<I',v))
        subprocess.run(['st-flash','write','/tmp/wb.bin','0x40007000'], capture_output=True)
    time.sleep(0.05)
    # Write BKP_DR1 = 0
    r = rram(0x40006C04)
    if r:
        d = bytearray(r); d[0]=0; d[1]=0
        open('/tmp/wb.bin','wb').write(bytes(d))
        subprocess.run(['st-flash','write','/tmp/wb.bin','0x40006C04'], capture_output=True)
    time.sleep(0.05)
    r = rram(0x40006C04,2)
    bkp = struct.unpack('<H',r)[0] if r else 0xFFFF
    return bkp == 0  # True = cleared

def reset_and_wait_for_app(timeout=5.0):
    """Clear BKP, reset Blue Pill, wait for app to start."""
    clear_bkp()
    subprocess.run(['st-flash','reset'], capture_output=True)
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        time.sleep(0.2)
        d = rram(0xE000ED08)
        if d:
            vtor = struct.unpack('<I',d)[0]
            if vtor == 0x08002000:
                return True, time.monotonic()-t0
    return False, timeout

def open_slcan():
    s = serial.Serial(PORT, 115200, timeout=0.5)
    time.sleep(0.3)
    while s.in_waiting: s.read(s.in_waiting)
    s.write(b'C\r'); time.sleep(0.3); s.read(256)
    s.write(b'S4\r'); time.sleep(0.3); s.read(256)
    s.write(b'O\r'); time.sleep(0.5); s.read(256)
    return s

def collect_frames(s, duration=1.5):
    frames = []; stop = [False]
    def m(s):
        while not stop[0]:
            b = s.read(256)
            if b: frames.append(b)
    t = threading.Thread(target=m, args=(s,), daemon=True); t.start()
    time.sleep(duration); stop[0] = True; time.sleep(0.1)
    return b''.join(frames)

# ─────────────────────────────────────────────────────────────
print("="*60)
print("3-NODE CAN BUS TEST (all pins connected)")
print("Pico2(XL2515) ↔ MCP2515(SPI) ↔ Blue Pill(bxCAN PB8/PB9)")
print("="*60)

# Stop slcand - it holds ttyACM1 and blocks all direct SLCAN tests
print("\n[INIT] Stopping slcand if running...")
slcand_was_running = False
r = subprocess.run(['pgrep', '-a', 'slcand'], capture_output=True, text=True)
for proc_line in r.stdout.strip().split('\n'):
    if 'ttyACM1' in proc_line or 'slcan' in proc_line:
        parts = proc_line.split()
        if parts:
            subprocess.run(['kill', parts[0]], capture_output=True)
            slcand_was_running = True
            print(f"  Killed slcand PID={parts[0]}")
if not slcand_was_running:
    subprocess.run(['killall', 'slcand'], capture_output=True)
subprocess.run(['ip', 'link', 'set', 'slcan0', 'down'], capture_output=True)
time.sleep(0.5)

# OTA latest firmware (with echo) — inline, no subprocess wrapper
print("\n[SETUP] OTA latest firmware...")
import importlib.util, os
ota_path = f'{BCG}/tools/can_ota_bluepill.py'
spec = importlib.util.spec_from_file_location('can_ota', ota_path)
can_ota = importlib.util.module_from_spec(spec)
spec.loader.exec_module(can_ota)

try:
    s_ota = can_ota.slcan_open(PORT)
    clear_bkp()
    subprocess.run(['st-flash','reset'], capture_output=True)
    time.sleep(0.3)
    uid0, _ = can_ota.wait_for_hello(s_ota, timeout=5.0)
    if uid0 is None: raise RuntimeError("No bootloader hello")
    fw_data = open(f'{BCG}/firmware/blue_pill_can_tool/blue_pill_can_tool.bin','rb').read()
    ota_ok = can_ota.upload_firmware(s_ota, fw_data, uid0)
    s_ota.close()
except Exception as e:
    print(f"  OTA exception: {e}")
    ota_ok = False
print(f"  OTA: {'SUCCESS ✓' if ota_ok else 'FAILED ✗'}")
if not ota_ok: sys.exit(1)

# Wait for app
print("\n[SETUP] Waiting for app after OTA...")
clear_bkp()
subprocess.run(['st-flash','reset'], capture_output=True)
time.sleep(2.5)
d = rram(0xE000ED08)
vtor = struct.unpack('<I',d)[0] if d else 0
print(f"  VTOR=0x{vtor:08X} {'APP ✓' if vtor==0x08002000 else 'BOOTLOADER - wait longer'}")

results = {}
N = 20

# ─── PHASE A: Pico2 TX → bxCAN RX (via echo) ─────────────────
print(f"\n{'─'*60}")
print(f"PHASE A: Pico2(XL2515) TX → Blue Pill(bxCAN) RX")
print(f"  Send ID=0x300; expect echo on ID=0x301 (proves bxCAN RX)")
s = open_slcan()
frm_a = []; stop_a = [False]
def m_a(s):
    while not stop_a[0]:
        b = s.read(256)
        if b: frm_a.append(b)
threading.Thread(target=m_a, args=(s,), daemon=True).start()

for i in range(N):
    s.write(f't3001{i:02X}\r'.encode())
    time.sleep(0.030)  # 30ms gap

time.sleep(1.5); stop_a[0] = True; time.sleep(0.1)
all_a = b''.join(frm_a)
z_a = all_a.count(b'z\r')
echo_a = [l for l in all_a.split(b'\r') if l.startswith(b't301')]  # echo id+1
results['A_ack'] = z_a
results['A_echo'] = len(echo_a)
ef_a_v, ef_a = 0, 'CLEAN ✓'
s.write(b'F\r'); time.sleep(0.3); ef_raw = s.read(20).strip()
try:
    v = int(ef_raw.decode().lstrip('F').rstrip('\r'), 16)
    ef_a_v = v
    bits = {0:'EWRN',1:'RXWRN',2:'TXWRN',3:'RXEP',4:'TXEP',5:'TXBO',6:'R0OVR',7:'R1OVR'}
    ef_a = f"0x{v:02X}=[{','.join(bits[i] for i in range(8) if v&(1<<i))}]" if v else 'CLEAN ✓'
except: pass
s.close()
print(f"  Pico2 TX ACK:    {z_a}/{N} ({z_a/N*100:.0f}%)")
print(f"  bxCAN echo(0x301): {len(echo_a)}/{N} ({len(echo_a)/N*100:.0f}%) {'✓' if len(echo_a)>=N*0.97 else '✗'}")
print(f"  EFLG: {ef_a}")

# ─── PHASE B: bxCAN bootloader TX → Pico2 RX ─────────────────
print(f"\n{'─'*60}")
print(f"PHASE B: Blue Pill(bxCAN) TX → Pico2(XL2515) RX")
s = open_slcan()
frm_b = []; stop_b = [False]
def m_b(s):
    while not stop_b[0]:
        b = s.read(256)
        if b: frm_b.append(b)
threading.Thread(target=m_b, args=(s,), daemon=True).start()
clear_bkp()
subprocess.run(['st-flash','reset'], capture_output=True)
time.sleep(3.5)
stop_b[0] = True; time.sleep(0.1)
all_b = b''.join(frm_b)
hello_b = [l for l in all_b.split(b'\r') if l.startswith(b't7DE')]
results['B'] = len(hello_b)
ef_b = 'CLEAN ✓'
s.write(b'F\r'); time.sleep(0.3); ef_raw = s.read(20).strip()
try:
    v = int(ef_raw.decode().lstrip('F').rstrip('\r'), 16)
    ef_b = f"0x{v:02X}" if v else 'CLEAN ✓'
except: pass
s.close()
print(f"  bxCAN hello(0x7DE): {len(hello_b)}/10 {'✓' if len(hello_b)>=8 else '✗'}")
print(f"  EFLG: {ef_b}")

# ─── PHASE C: MCP2515 TX → Pico2 RX ──────────────────────────
print(f"\n{'─'*60}")
print(f"PHASE C: MCP2515(SPI) TX → Pico2 RX + bxCAN echo")
print(f"  Send ID=0x600 → bxCAN lazy-inits MCP2515 → MCP2515 TX 0x601")

# Wait for app after bootloader
clear_bkp()
subprocess.run(['st-flash','reset'], capture_output=True)
time.sleep(3.5)
d = rram(0xE000ED08); vtor = struct.unpack('<I',d)[0] if d else 0
print(f"  App state: VTOR=0x{vtor:08X} {'✓' if vtor==0x08002000 else '?'}")

s = open_slcan()
frm_c = []; stop_c = [False]
def m_c(s):
    while not stop_c[0]:
        b = s.read(256)
        if b: frm_c.append(b)
threading.Thread(target=m_c, args=(s,), daemon=True).start()

for i in range(N):
    s.write(f't6001{i:02X}\r'.encode())
    time.sleep(0.050)  # 50ms - first trigger does lazy init (~25ms)

time.sleep(2.0); stop_c[0] = True; time.sleep(0.1)
all_c = b''.join(frm_c)
mcp_tx = [l for l in all_c.split(b'\r') if l.startswith(b't601')]
diag = [l for l in all_c.split(b'\r') if l.startswith(b't602')]
echo_c = [l for l in all_c.split(b'\r') if l.startswith(b't601')]  # echo from MCP2515 TX
z_c = all_c.count(b'z\r')
results['C_diag'] = len(diag)
results['C_mcp'] = len(mcp_tx)
ef_c = 'CLEAN ✓'
s.write(b'F\r'); time.sleep(0.3); ef_raw = s.read(20).strip()
try:
    v = int(ef_raw.decode().lstrip('F').rstrip('\r'), 16)
    ef_c = f"0x{v:02X}" if v else 'CLEAN ✓'
except: pass
s.close()

print(f"  Trigger ACK(z): {z_c}/{N}")
print(f"  SPI diag(0x602): {len(diag)}")
for l in diag[:2]:
    try:
        data = bytes.fromhex(l[5:11].decode())
        print(f"    init_ok={data[1]} CANSTAT=0x{data[2]:02X} "
              f"({'NORMAL ✓' if data[2]==0 else 'CONFIG/DEAD - SPI NOT CONNECTED' if data[2]==0x80 else f'UNKNOWN 0x{data[2]:02X}'})")
    except: pass
print(f"  MCP2515 TX(0x601)→Pico2: {len(mcp_tx)}/{N} ({len(mcp_tx)/N*100:.0f}%) {'✓' if len(mcp_tx)>=N*0.97 else '?'}")
print(f"  EFLG: {ef_c}")

# ─── PHASE D: Burst + R0OVR fix ───────────────────────────────
print(f"\n{'─'*60}")
print(f"PHASE D: Burst test (3ms) → R0OVR overflow fix verification")
s = open_slcan()
for i in range(100): s.write(f't1001{i&0xFF:02X}\r'.encode()); time.sleep(0.003)
time.sleep(1.5)  # wait for all echo frames to arrive and settle
while s.in_waiting: s.read(s.in_waiting); time.sleep(0.1)  # drain buffer
s.write(b'F\r'); time.sleep(0.5)
# Read until we see the 'F' response (starts with 'F' or is just the hex)
ef_d_raw = b''
deadline_d = time.monotonic() + 2.0
while time.monotonic() < deadline_d:
    chunk = s.read(64)
    if chunk: ef_d_raw += chunk
    # Look for 'F' command response: 'F' followed by hex digits and '\r'
    for token in ef_d_raw.split(b'\r'):
        t = token.strip()
        if t and t[0:1] == b'F' and len(t) <= 4:
            ef_d = t; break
    else:
        continue
    break
else:
    ef_d = ef_d_raw[:20].strip()
try:
    v = int(ef_d.decode().lstrip('F').rstrip('\r'), 16)
    ef_d_str = f"EFLG=0x{v:02X} CLEAN ✓" if v==0 else f"EFLG=0x{v:02X} (R0OVR not cleared)"
except: ef_d_str = f"raw={ef_d}"
print(f"  100@3ms EFLG: {ef_d_str}")
results['D'] = ef_d_str
s.close()

# ─── SUMMARY ──────────────────────────────────────────────────
print(f"\n{'='*60}")
print(f"3-NODE CAN FINAL RESULTS")
print(f"{'='*60}")
rA_ack = results.get('A_ack', 0)/N*100
rA_echo = results.get('A_echo', 0)/N*100
rB = results.get('B', 0)/10*100
rC_diag = results.get('C_diag', 0)
rC_mcp = results.get('C_mcp', 0)/N*100

print(f"  {'Pico2 TX→bxCAN ACK':<35} {rA_ack:>5.0f}%  {'✓' if rA_ack>=97 else '✗'}")
print(f"  {'Pico2 TX→bxCAN RX (echo)':<35} {rA_echo:>5.0f}%  {'✓' if rA_echo>=97 else '✗'}")
print(f"  {'bxCAN TX→Pico2 RX':<35} {rB:>5.0f}%  {'✓' if rB>=80 else '✗'}")
print(f"  {'MCP2515 SPI diag frames':<35} {rC_diag:>5}   {'✓' if rC_diag>0 else '? (SPI not wired?)'}")
print(f"  {'MCP2515 TX→Pico2 RX':<35} {rC_mcp:>5.0f}%  {'✓' if rC_mcp>=97 else '? (SPI/bus issue)'}")
print(f"  {'Overflow R0OVR fix':<35} {'PASS ✓' if 'CLEAN' in results.get('D','') else 'FAIL ✗':>6}")
print()
all_pass = rA_ack>=97 and rA_echo>=97 and rB>=80
print(f"  bxCAN TX/RX: {'ALL PASS ✓' if all_pass else 'SOME FAIL'}")
if rC_diag == 0:
    print(f"  MCP2515 SPI: NOT CONNECTED (PA4-PA7/PB0 wiring required)")
elif rC_mcp < 50:
    print(f"  MCP2515 TX: LOW RATE — check MCP2515 CANH/CANL on bus")
else:
    print(f"  MCP2515 TX: {'WORKING ✓' if rC_mcp>=97 else 'PARTIAL'}")

# Restart slcand to restore SocketCAN interface for ROS2
print("\n[CLEANUP] Restarting slcand...")
try:
    subprocess.Popen(['slcand', '-o', '-s4', '-t', 'hw', '-S', '115200', '/dev/ttyACM1', 'slcan0'])
    time.sleep(1.0)
    subprocess.run(['ip', 'link', 'set', 'slcan0', 'up'], capture_output=True)
    print("  slcand restarted ✓")
except Exception as e:
    print(f"  slcand restart failed: {e}")
