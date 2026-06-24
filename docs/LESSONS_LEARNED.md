# Lessons Learned: STM32 CAN OTA Firmware Update

From a real project: 31 bugs found across 9 audit rounds (~120 subagents).

---

## 1. Use Native bxCAN Instead of MCP2515+SPI

**Category:** Hardware  
**Problem:** MCP2515 over SPI requires 5V VCC for reliable operation, but STM32 SPI is 3.3V. At 5V VCC, VIH_min = 3.5V > STM32 output 3.3V → SPI writes silently fail.  
**Solution:** Use STM32F103's built-in bxCAN peripheral on PB8 (RX) / PB9 (TX) with AFIO remap.

```c
// WRONG: MCP2515 at 5V, SPI unreliable
mcp_write_reg(TXB0CTRL, 0x08);  // TXREQ write fails silently!

// RIGHT: bxCAN inside STM32, no SPI needed
CAN1->TXB[0].TIR |= CAN_TIR_TXRQ;  // always works
```

**Key insight:** The bxCAN peripheral runs at 3.3V alongside the CPU — no external chip, no voltage level issues.

---

## 2. MCP2515 at 3.3V VCC Works Correctly

**Category:** Hardware  
**Problem:** Was at 5V VCC for TJA1050 drive strength.  
**Confirmation:** At 3.3V VCC: TXB0CTRL=0x00 after TX (TXREQ cleared = Pico 2 ACKed). All SPI writes reliable.  
**Rule:** If using MCP2515 with a 3.3V MCU, **always power MCP2515 at 3.3V**. TJA1050 produces slightly weaker differential at 3.3V but works for typical cable lengths (<1m).

---

## 3. TXREQ Is Bit 2 (0x04), Not Bit 3 (0x08)

**Category:** Firmware  
**Problem:** Many libraries define `TXREQ = 0x08` (bit 3). The official MCP2515 datasheet (DS21801J) specifies TXREQ = bit 2 (0x04).  
**Effect:** TX was never triggered; all CAN transmissions silently failed.

```c
// WRONG (common in many libs):
#define MCP_TXCTRL_TXREQ  0x08   // bit 3 = TXERR!

// CORRECT (DS21801J):
#define MCP_TXCTRL_TXREQ  0x04   // bit 2
#define MCP_TXCTRL_TXERR  0x08   // bit 3
#define MCP_TXCTRL_MLOA   0x10   // bit 4
#define MCP_TXCTRL_ABTF   0x20   // bit 5
```

---

## 4. One-Shot Mode vs Standard Mode

**Category:** Firmware  
**Problem:** Without OSM (One-Shot Mode), a failed TX causes the MCP2515/bxCAN to retry indefinitely → bus flooding → bus-off state → all communication stops.  
**Rule:** Use One-Shot Mode in the bootloader (host controls retries). Use Standard Mode in the app.

```c
// Bootloader (OTA): One-Shot — host retries if needed
bxcan_init(1);   // osm=1: CAN_MCR_NART set

// App (normal operation): Standard — auto-retry for reliable delivery
bxcan_app_init(250000, 0);  // osm=0: auto-retry
```

For MCP2515: `CANCTRL.OSM = bit3`. For bxCAN: `CAN_MCR.NART = bit4`.

---

## 5. bxCAN FIFO Depth = 3 Frames

**Category:** Protocol  
**Problem:** Sending OTA page data at 0.3ms inter-frame caused FIFO overflow (depth=3). CRC failed on every page — OTA unusable.  
**Solution:** Use 2ms inter-frame delay for page upload at 250kbps.

```python
# OTA page upload — prevent bxCAN FIFO overflow
for f in range(128):
    slcan_send(s, BL_DATA_ID, page_data[f*8:(f+1)*8])
    time.sleep(0.002)  # 2ms gap = ~15× CAN frame time, safe for FIFO depth=3
```

---

## 6. OTA Page Protocol: All 1024 Bytes Must Be Sent

**Category:** Protocol  
**Problem:** Original protocol sent 1020 bytes per page (last 4 bytes silently dropped). Resulted in corrupted firmware — every page had wrong data at offsets 1020-1023.  
**Solution:** 129 frames per page: 128 data frames × 8 bytes = 1024 bytes, then 1 CRC frame.

```c
// Bootloader can_upload() — correct protocol:
for (uint32_t f = 0; f < 129; f++) {
    // ... receive frame ...
    if (f == 128) {
        master_crc = tmp[0] | (tmp[1]<<8) | (tmp[2]<<16) | (tmp[3]<<24);
    } else {
        // Critical: use uint32_t, NOT uint8_t, for (1024 - byte_idx)
        // uint8_t overflow: (uint8_t)(1024-0) = 0 → copies nothing!
        uint32_t remain = 1024U - byte_idx;
        uint8_t take = (rlen < remain) ? rlen : (uint8_t)remain;
        for (uint8_t b = 0; b < take; b++) page_buf[byte_idx++] = tmp[b];
    }
}
```

---

## 7. `uint8_t` Overflow in Can Upload — Catastrophic Bug

**Category:** Firmware  
**Problem:** `uint8_t take = (rlen < (uint8_t)(1024 - byte_idx)) ? rlen : (uint8_t)(1024 - byte_idx)` — when `byte_idx=0`: `(uint8_t)1024 = 0` → copies ZERO bytes on every single frame. CRC always fails. CAN OTA completely non-functional despite all other fixes.  
**Fix:** Cast to `uint32_t` first, then do the comparison.

```c
// BROKEN: (uint8_t)(1024 - 0) = 0!
uint8_t take = (rlen < (uint8_t)(1024 - byte_idx)) ? rlen : (uint8_t)(1024 - byte_idx);

// CORRECT:
uint32_t remain = 1024U - byte_idx;
uint8_t  take   = (rlen < remain) ? rlen : (uint8_t)remain;
```

---

## 8. Pico 2 SLCAN Bitrate Table: Missing S3=100k

**Category:** Tooling  
**Problem:** Old bitrate table `{10k,20k,50k,125k,250k,...}` was missing 100k. All entries S3+ were shifted. `S5` sent `250k` per SLCAN standard but mapped to `500k` in firmware → silent bitrate mismatch → all OTA attempts failed.  
**Fix:** Insert `100000u` at index 3:

```c
static const uint32_t bitrates[] = {
    10000u, 20000u, 50000u, 100000u, 125000u, 250000u, 500000u, 800000u, 1000000u
};
// Now: S5 = 250kbps (SLCAN standard) ✓
```

---

## 9. SLCAN Frame Terminator: `\r` not `\n`

**Category:** Tooling  
**Problem:** Python's `readline()` waits for `\n` but SLCAN uses `\r`. OTA tool blocked for full timeout on every call → never received frames.  
**Fix:** Read byte-by-byte until `\r`:

```python
def _read_until_cr(s, timeout):
    deadline = time.monotonic() + timeout
    buf = b''
    while time.monotonic() < deadline:
        s.timeout = max(0.0, min(deadline - time.monotonic() + 0.01, 0.1))
        ch = s.read(1)
        if not ch: continue
        if ch == b'\r': return buf
        buf += ch
    return buf
```

---

## 10. Remote CAN Reboot Trigger

**Category:** Protocol  
**Problem:** Needed a way to reboot Blue Pill from bootloader without physical access.  
**Solution:** App watches for magic CAN frame and writes BKP register before reset.

```c
// App main loop — CAN reboot trigger
if (id == 0x7FFU && dlc >= 3 &&
    data[0] == 0xB0U && data[1] == 0x01U && data[2] == 0xB2U) {
    RCC_APB1ENR |= (1U<<28)|(1U<<27);  // PWREN + BKPEN
    PWR_CR |= (1U<<8);                  // DBP — disable BKP write protection
    BKP_DR1 = 0xB001U;                  // magic for 30s OTA window
    NVIC_SystemReset();
}
```

Bootloader checks `BKP_DR1 == 0xB001` at startup → extends hello window from 500ms to 30 seconds.

---

## 11. bxCAN Listen-Only: SILM (bit31), Not LBKM (bit30)

**Category:** Firmware  
**Problem:** Used `CAN1->BTR |= (1U<<30)` thinking that was silent/listen-only mode. Bit 30 is LBKM (Loop-Back Mode) — TX pin still drives bus, ACKs still generated.  
**Fix:** Bit 31 = SILM (Silent Mode) = true listen-only.

```c
// WRONG — loopback mode, still drives bus:
CAN1->BTR |= (1U<<30);  // LBKM

// CORRECT — true listen-only, CAN_TX stays recessive:
CAN1->BTR |= (1U<<31);  // SILM
```

---

## 12. MCP2515 CNF Values @ 8MHz Crystal

Confirmed working, **250kbps**: CNF1=0x00, CNF2=0x9E, CNF3=0x03 (16Tq, 81.25% SP).

| Rate | CNF1 | CNF2 | CNF3 | TQ | SP% | Notes |
|------|------|------|------|-----|-----|-------|
| 1Mbps | 0x00 | 0x80 | 0x00 | 4 | 75% | Min TQ, marginal |
| 500k | 0x00 | 0x90 | 0x02 | 8 | 75% | |
| 250k | 0x00 | 0x9E | 0x03 | 16 | 81.25% | **Confirmed ✓** |
| 125k | 0x01 | 0x9E | 0x03 | 16 | 81.25% | |
| 100k | 0x01 | 0xAD | 0x02 | 16 | 81.25% | |
| 50k | 0x04 | 0x9E | 0x03 | 16 | 81.25% | |
| 20k | 0x0B | 0x9E | 0x03 | 16 | 81.25% | ~20.8k (±4%) |
| 10k | 0x18 | 0x9E | 0x03 | 16 | 81.25% | |

CNF2=0x9E: BTLMODE=1, PHSEG1=3(4Tq), PRSEG=6(7Tq). CNF3=0x03: PHSEG2=3(4Tq).

---

## 13. Always Flush Input Buffer Before OTA Protocol

**Category:** Tooling  
**Problem:** Stale 'D' frame from a previous failed OTA session sitting in OS serial buffer triggered the "done" branch on page 0 without flashing anything.  
**Fix:** Call `s.reset_input_buffer()` at start of `upload_firmware()`.

---

## 14. Bitrate Mismatch Is Silent

**Category:** Debugging  
**Problem:** EFLG=F00 (no errors) even when Pico 2 was at 500kbps and Blue Pill at 250kbps. "No errors" does NOT mean "communicating". It can mean "different bitrates, ignoring each other."  
**Debugging method:** Check `rx_count` via SWD to confirm frames are actually received, not just check EFLG.

---

## 15. slcand Holds ttyACMx — Must Stop Before Direct SLCAN Access

**Category:** Tooling / System Integration  
**Problem:** The ROS2 stack (or any SocketCAN setup) runs `slcand` to bridge the Pico 2 USB SLCAN port to a `slcan0` kernel network interface. Any Python script that opens `ttyACM1` directly (OTA tool, test scripts) fails with `ENOTTY (error 25, Inappropriate ioctl for device)` because slcand holds an exclusive file descriptor.  
**Fix:** Stop slcand before test, restart after.

```python
import subprocess, time

# Stop slcand (releases ttyACM1)
subprocess.run(['killall', 'slcand'], capture_output=True)
time.sleep(0.5)
subprocess.run(['ip', 'link', 'set', 'slcan0', 'down'], capture_output=True)

# ... run OTA / direct SLCAN tests ...

# Restart slcand (restores SocketCAN for ROS2)
subprocess.Popen(['slcand', '-o', '-s4', '-t', 'hw', '-S', '115200', '/dev/ttyACM1', 'slcan0'])
time.sleep(0.8)
subprocess.run(['ip', 'link', 'set', 'slcan0', 'up'], capture_output=True)
```

**Key insight:** `pgrep -a slcand` shows whether it is running and which port it holds.

---

## 16. MCP2515 Lazy-Init Diagnostic Frame Only Fires Once

**Category:** Firmware / Testing  
**Problem:** The app sends a SPI diagnostic frame (0x602) only the **first** time MCP2515 is initialised (lazy init on first 0x600 trigger). Subsequent runs with `g_mcp_initialized=1` skip the diagnostic. A test that checks CANSTAT via the 0x602 frame will falsely report failure on the second run even though MCP2515 TX is working fine (0x601 arrives 10/10).  
**Fix:** Use the TX frame count as the primary health metric. Treat CANSTAT as supplementary info available only on first init.

```python
# WRONG — fails on second run:
ok = (canstat == 0x00) and (mcp_tx_recv == N)

# CORRECT — primary metric is actual TX success:
ok = mcp_tx_recv >= N * 0.90
# CANSTAT=0x00 from 0x602 is bonus confirmation on first run only
```

---

## 17. Drain Input Buffer Before EFLG 'F' Query

**Category:** Tooling  
**Problem:** After a burst TX test (100 frames @ 3ms = 300ms of traffic) the RX buffer contains hundreds of queued ACK ('z') and echo frames. Sending 'F\r' then immediately reading 20 bytes returns the start of those queued frames, not the EFLG response.  
**Fix:** Drain the buffer completely before sending 'F'.

```python
# After burst TX:
time.sleep(1.5)                    # let all echo frames arrive
while s.in_waiting:
    s.read(s.in_waiting)
    time.sleep(0.1)                # re-check: USB may deliver in batches
s.write(b'F\r')
time.sleep(0.5)
# Then read looking specifically for token starting with 'F':
for token in s.read(64).split(b'\r'):
    t = token.strip()
    if t and t[:1] == b'F' and len(t) <= 4:
        eflg = int(t[1:], 16); break
```

---

## 18. OTA in Test Scripts: Import Module, Don't Subprocess

**Category:** Tooling  
**Problem:** Launching `can_ota_bluepill.py` as a `subprocess.Popen` with `stdout=PIPE` from a test harness creates a race: the outer script must reset the Blue Pill at the right moment, but the subprocess buffers its own output (not a TTY → block-buffered) so the outer loop sees nothing until the process exits. Also, if slcand was running and the outer script forgot to kill it before `Popen`, the subprocess fails immediately.  
**Fix:** Import the OTA module directly with `importlib` — runs in-process, shares the already-open serial handle, no timing race.

```python
import importlib.util
spec = importlib.util.spec_from_file_location('can_ota', '/path/to/can_ota_bluepill.py')
can_ota = importlib.util.module_from_spec(spec)
spec.loader.exec_module(can_ota)

s = can_ota.slcan_open(PORT)        # open once
uid0, _ = can_ota.wait_for_hello(s, timeout=5.0)
fw = open('firmware.bin', 'rb').read()
ok = can_ota.upload_firmware(s, fw, uid0)
s.close()
```

---

## 19. BKP Register Clear in Automated Tests

**Category:** Testing  
**Problem:** An app that was previously triggered to reboot-to-bootloader leaves `BKP_DR1=0xB001` set. The next automated test reset sees the magic value and enters the 30-second bootloader window instead of the 500ms one. The test hangs for 30s or times out while waiting for the app to start.  
**Fix:** Every automated reset must clear BKP_DR1 via SWD before calling `st-flash reset`.

```python
def clear_bkp():
    def w32(a, v):
        open('/tmp/wb.bin','wb').write(struct.pack('<I', v))
        subprocess.run(['st-flash','write','/tmp/wb.bin',f'0x{a:08X}'], capture_output=True)
    def r32(a):
        subprocess.run(['st-flash','read','/tmp/rb.bin',f'0x{a:08X}','4'], capture_output=True)
        d = open('/tmp/rb.bin','rb').read()[:4]
        return struct.unpack('<I', d)[0] if len(d) == 4 else 0
    w32(0x4002101C, r32(0x4002101C) | (3 << 27))  # RCC: PWREN + BKPEN
    time.sleep(0.05)
    w32(0x40007000, r32(0x40007000) | (1 << 8))    # PWR_CR: DBP
    time.sleep(0.05)
    w32(0x40006C04, 0)                              # BKP_DR1 = 0
    time.sleep(0.05)
```

---

## 20. bxCAN Echo Technique — Prove RX Without SWD

**Category:** Debugging  
**Problem:** Reading `g_rx_count` via SWD to verify bxCAN receive is unreliable — variable addresses shift between builds (BSS layout changes). The address read via SWD may not match the compiled binary.  
**Fix:** Add an echo in the app: transmit `id+1` for every received frame. The Pico 2 sees the echo frame → proves bxCAN RX is working, no SWD needed.

```c
// App main loop — CAN RX echo (diagnostic)
if (bxcan_app_rx(&id, data, &dlc, &ext)) {
    bxcan_app_tx(id + 1U, data, dlc, ext);  // echo on id+1
    // ... rest of handling ...
}
// Send 0x300, expect 0x301 back → bxCAN RX confirmed ✓
```

---

## Summary Table

| Bug # | Severity | Component | Issue | Fix |
|-------|----------|-----------|-------|-----|
| 1-2 | Critical | OTA page protocol | 1020B/page, uint8_t overflow | 1024B, uint32_t |
| 3 | Critical | Pico 2 SLCAN | S5=500k (missing 100k in table) | Insert 100k at S3 |
| 4 | High | OTA tool | readline blocks on \r | \r-aware reader |
| 5 | High | mcp2515 | TXREQ=0x08 (wrong bit) | 0x04 per DS21801J |
| 6 | High | All CAN | No OSM → bus-off | NART/OSM for bootloader |
| 7 | High | Bootloader | UID2 match (should be UID0) | DESIG_UNIQUE_ID0 |
| 8 | High | bxCAN | LBKM not SILM for listen | bit31 not bit30 |
| 9 | High | bxCAN | bxCAN FIFO overflow | 2ms inter-frame |
| 10 | High | OTA tool | Missing 'D' handler in loop | elif ack=='D' |
| 11-14 | Med/Low | Various | Comments, types, edge cases | See git log |
| 15 | High | System | slcand blocks ttyACMx | Kill before test, restart after |
| 16 | Med | Testing | Lazy-init diag only fires once | Use TX count as primary metric |
| 17 | Med | Testing | Buffer not drained before EFLG | Drain then send 'F' |
| 18 | High | Tooling | Subprocess OTA port conflict | Import module inline |
| 19 | Critical | Testing | BKP=0xB001 causes 30s BL window | Clear BKP before every reset |
| 20 | Med | Debugging | SWD rx_count address shifts | bxCAN echo (id+1) instead |
