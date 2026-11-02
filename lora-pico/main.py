# lora_to_uart_fire_and_forget.py
# Simple LoRa -> UART forwarder (no ACK wait). Uses UART0 (GP0 TX, GP1 RX).

import time, machine, ubinascii, ujson as json, micropython, gc
from ulora import LoRa, SPIConfig

# config
RFM95_RST = 9; RFM95_CS = 8; RFM95_INT = 22
RF95_FREQ = 915.0; SERVER_ADDRESS = 2
# uartfix3: the bridge only transmits ACKs. Request 17 -> PA_BOOST, no PA_DAC,
# OutputPower 12 -> about +14 dBm out (was 20 -> about +17 dBm). Nodes are heard
# at -44..-74 dBm, so a 3 dB cut leaves > 40 dB of SF7 margin for the ACK.
RF95_POW = 17
UART_ID = 0; UART_TX_PIN = 0; UART_RX_PIN = 1; UART_BAUD = 230400

# LED Setup
LED_PIN_UART = 13
LED_PIN_LORA = 12
led_uart = machine.Pin(LED_PIN_UART, machine.Pin.OUT)
led_lora = machine.Pin(LED_PIN_LORA, machine.Pin.OUT)

# uartfix3: nothing is printed after boot unless VERBOSE. On rp2 a print to a
# connected-but-wedged USB CDC waits for the CDC TX timeout before it drops the
# text, which stalls the main loop and delays ACKs. Health goes out as a
# periodic STATUS frame on the same UART -> Wiznet -> Node-RED instead.
VERBOSE = False
STATUS_EVERY_MS = 60000          # STATUS frame cadence
STATUS_FIRST_MS = 5000           # first STATUS shortly after boot
STATUS_FILE = "/bridge_status.txt"   # overwritten, not appended
STATUS_FILE_EVERY_MS = 1800000   # 48 small writes/day; littlefs wear is negligible
_boot_ms = time.ticks_ms()
_frames_tx = 0                   # frames handed to _uart_write_all
_bytes_tx = 0
_wr_ms_max = 0
_rxq_drop = 0                    # RX queue overflow
_dup_total = 0                   # same header_from+id within 30 s (node retry / repeater copy)
_dup_by_src = {}                 # src -> later copies with no hop stamp (= node retry, ACK missed)
_rpt_by_src = {}                 # src -> later copies with FLAGS_HOP_SET 0x08 (repeater echo)
_DUP_SRC_MAX = 12
_rx_by_src = {}                  # src -> first copies forwarded, capped
_status_seq = 0
try:
    _reset_cause = machine.reset_cause()
except Exception:
    _reset_cause = -1

def _log(*a):
    if VERBOSE:
        try:
            print(*a)
        except Exception:
            pass

def _bump(d, k):
    # str keys: MicroPython json.dumps writes int dict keys unquoted (invalid JSON)
    k = str(k)
    if k in d:
        d[k] += 1
    elif len(d) < _DUP_SRC_MAX:
        d[k] = 1
    else:
        d["other"] = d.get("other", 0) + 1

# small queue
_payload_queue = []
_QUEUE_MAX = 32
_led_lora_pending = False

# UART dedupe. Key is RadioHead header_from + header_id. Short memory so an
# 8-bit id can be reused; long enough that a repeater echo is a duplicate.
# Does not suppress ACKs. Those stay queued in ulora and still TX for each RX.
_seen_keys = []
_SEEN_MAX = 32
_SEEN_MS = 30000

def _drop_uart_dup(payload):
    try:
        src = int(payload.header_from) & 0xff
        pid = int(payload.header_id) & 0xff
    except Exception:
        return False
    now = time.ticks_ms()
    fresh = []
    dup = False
    for item in _seen_keys:
        if time.ticks_diff(now, item[2]) > _SEEN_MS:
            continue
        fresh.append(item)
        if item[0] == src and item[1] == pid:
            dup = True
    if not dup:
        fresh.append((src, pid, now))
        if len(fresh) > _SEEN_MAX:
            fresh = fresh[-_SEEN_MAX:]
    _seen_keys[:] = fresh
    return dup

# uart single instance
# uartfix2 (2026-10-19): rp2 uart.write() returns SHORT if the TX ring stays full
# longer than timeout_char. Unset, timeout_char = 13000/baud+1 = 1 ms at 230400,
# but the TX IRQ only refills the FIFO every ~28 chars (1.2 ms), so with 1 ms
# tick rounding a frame > ~288 B (256 ring + 32 FIFO) is cut at 292/323/354 B
# and the tail is never sent. Fix: whole frame fits the ring (txbuf=1024), a sane
# timeout_char, and a write-all loop that checks the returned count.
UART_TXBUF = 1024
UART_TIMEOUT_CHAR_MS = 20
uart = machine.UART(UART_ID, UART_BAUD, tx=machine.Pin(UART_TX_PIN), rx=machine.Pin(UART_RX_PIN),
                    timeout=10, timeout_char=UART_TIMEOUT_CHAR_MS, txbuf=UART_TXBUF)
_uart_short_writes = 0   # write() calls that returned < requested
_uart_short_frames = 0   # frames not fully queued before the deadline

def _enqueue_payload(payload):
    global _rxq_drop
    try:
        if len(_payload_queue) >= _QUEUE_MAX:
            _payload_queue.pop(0)  # drop oldest under burst
            _rxq_drop += 1
        _payload_queue.append(payload)
    except Exception:
        pass

def _scheduled_enqueue(arg):
    _enqueue_payload(arg)

def on_recv_callback(payload):
    # Called from ulora IRQ context — never sleep or do heavy work here.
    try:
        micropython.schedule(_scheduled_enqueue, payload)
        global _led_lora_pending
        _led_lora_pending = True
    except Exception:
        try:
            _enqueue_payload(payload)
        except Exception:
            pass

def _uart_write_all(buf, deadline_ms=200):
    # Loop on the returned count (memoryview: no copies) until all of buf is in
    # the TX ring. Returns bytes queued; == len(buf) unless the deadline expired.
    global _uart_short_writes
    mv = memoryview(buf)
    n = len(buf)
    off = 0
    t0 = time.ticks_ms()
    while off < n:
        try:
            w = uart.write(mv[off:])
        except OSError:
            w = 0           # EAGAIN: nothing queued this call
        if w:
            off += w
            if off < n:
                _uart_short_writes += 1
        elif time.ticks_diff(time.ticks_ms(), t0) > deadline_ms:
            break
    return off

def _frame_and_send(payload_bytes):
    global _uart_short_frames, _frames_tx, _bytes_tx, _wr_ms_max
    L = len(payload_bytes)
    # Match Wiznet FRAME_MAGIC path: A5 5A + uint16_be length + body
    frame = bytearray(4 + L)
    frame[0] = 0xA5; frame[1] = 0x5A; frame[2] = (L >> 8) & 0xFF; frame[3] = L & 0xFF
    frame[4:] = payload_bytes
    try:
        t0 = time.ticks_ms()
        sent = _uart_write_all(frame)
        dt = time.ticks_diff(time.ticks_ms(), t0)
        _frames_tx += 1
        _bytes_tx += sent
        if dt > _wr_ms_max:
            _wr_ms_max = dt
        if sent != len(frame):
            _uart_short_frames += 1
        _log("FF: wrote frame len", L, "written", sent, "of", len(frame), "ms", dt,
             "shortw", _uart_short_writes, "shortf", _uart_short_frames)
        led_uart.on()
        time.sleep(0.01)
        led_uart.off()
    except Exception as e:
        _log("FF: write error", e)

def _status_obj(lora):
    global _status_seq
    _status_seq += 1
    # Everything nested under one key: no header_from/header_id/meta/rssi/snr
    # at the top level, so Node-RED per-node/inventory/link-stats paths ignore it.
    return {"bridge_status": {
        "seq": _status_seq,
        "up_s": time.ticks_diff(time.ticks_ms(), _boot_ms) // 1000,
        "rst": _reset_cause,
        "fr": _frames_tx, "by": _bytes_tx,
        "sw": _uart_short_writes, "sf": _uart_short_frames, "wmax": _wr_ms_max,
        "rxq_drop": _rxq_drop, "q": len(_payload_queue),
        "rx": _rx_by_src, "dup": _dup_total, "dup_src": _dup_by_src, "rpt_src": _rpt_by_src,
        "aq": getattr(lora, "ack_q", -1), "aok": getattr(lora, "ack_ok", -1),
        "ato": getattr(lora, "ack_to", -1), "aerr": getattr(lora, "ack_err", -1),
        "adrop": getattr(lora, "ack_drop", -1), "alate": getattr(lora, "ack_late", -1),
        "alat_max": getattr(lora, "ack_lat_max", -1), "alat": getattr(lora, "ack_lat_last", -1),
        "txto": getattr(lora, "tx_to", -1),
        "pa": getattr(lora, "pa_regs", None), "pow": RF95_POW,
        "mf": gc.mem_free(),
    }}

def _send_status(lora, to_file):
    try:
        b = json.dumps(_status_obj(lora)).encode('utf-8')
        _frame_and_send(b)
        if to_file:
            try:
                with open(STATUS_FILE, "wb") as f:
                    f.write(b)
            except Exception:
                pass
    except Exception:
        pass

def run():
    lora = LoRa(SPIConfig.rp2_0, RFM95_INT, SERVER_ADDRESS, RFM95_CS,
                reset_pin=RFM95_RST, freq=RF95_FREQ, tx_power=RF95_POW, receive_all=True, acks=True)
    lora.on_recv = on_recv_callback
    try:
        lora.set_mode_rx()
    except Exception:
        pass

    # Boot banner only (before any radio TX). Nothing else prints unless VERBOSE.
    try:
        print("FF forwarder uartfix3 UART {} @ {} acks={} pow={} pa={}".format(
            UART_ID, UART_BAUD, getattr(lora, "_acks", None), RF95_POW, getattr(lora, "pa_regs", None)))
        print("FF uart", uart)   # shows txbuf / timeout_char actually applied
    except Exception:
        pass
    next_status = time.ticks_add(time.ticks_ms(), STATUS_FIRST_MS)
    next_file = time.ticks_add(time.ticks_ms(), STATUS_FIRST_MS)
    try:
        while True:
            # Drain RF ACKs outside IRQ (ulora queues them on RX) — before UART work
            try:
                lora.process_pending_acks()
            except Exception as e:
                _log("ACK drain err", e)
            global _led_lora_pending, _dup_total
            if _led_lora_pending:
                _led_lora_pending = False
                led_lora.on()
                time.sleep(0.01)
                led_lora.off()
            if _payload_queue:
                payload = _payload_queue.pop(0)
                if _drop_uart_dup(payload):
                    # Later copy, same header_from + header_id. First copy already went out.
                    # A node retry reuses header_id, so dup by src ~= ACKs that node missed
                    # (repeater copies also land here).
                    _dup_total += 1
                    try:
                        if int(payload.header_flags) & 0x08:
                            _bump(_rpt_by_src, int(payload.header_from) & 0xff)
                        else:
                            _bump(_dup_by_src, int(payload.header_from) & 0xff)
                    except Exception:
                        pass
                    _log("FF: drop dup from", int(payload.header_from), "id", int(payload.header_id))
                else:
                    try:
                        _bump(_rx_by_src, int(payload.header_from) & 0xff)
                    except Exception:
                        pass
                    try:
                        try:
                            message_str = payload.message.decode('utf-8')
                        except Exception:
                            message_str = ubinascii.b2a_base64(payload.message).decode().strip()
                        msg = {
                            "message": message_str,
                            "header_to": int(payload.header_to),
                            "header_from": int(payload.header_from),
                            "header_id": int(payload.header_id),
                            "header_flags": int(payload.header_flags),
                            "rssi": float(payload.rssi),
                            "snr": float(payload.snr),
                            "ts": time.time()
                        }
                    except Exception:
                        msg = {"error":"serialize_failed","raw":str(getattr(payload,"message",b""))}
                    payload_bytes = json.dumps(msg).encode('utf-8')
                    _frame_and_send(payload_bytes)
                # Drain again after UART so ACK isn't stuck behind framing
                try:
                    lora.process_pending_acks()
                except Exception as e:
                    _log("ACK drain err", e)
            else:
                now = time.ticks_ms()
                if time.ticks_diff(now, next_status) >= 0:
                    to_file = time.ticks_diff(now, next_file) >= 0
                    if to_file:
                        next_file = time.ticks_add(now, STATUS_FILE_EVERY_MS)
                    next_status = time.ticks_add(now, STATUS_EVERY_MS)
                    _send_status(lora, to_file)
                else:
                    time.sleep(0.01)
    finally:
        try:
            lora.close()
        except:
            pass

if __name__ == "__main__":
    run()
