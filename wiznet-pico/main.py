# main.py - Adaptive UART -> WebSocket forwarder (memory-hardened)
import time, socket, ubinascii, ujson as json, machine, gc

# SET ETHERNET ADDRESS VARIABLES HERE
IP_ADDRESS = '192.168.10.181'
SUBNET_MASK = '255.255.255.0'
GATEWAY = '192.168.10.1'
DNS = '192.168.10.1'

# LED SETUP
LED_PIN_WS = 25
led_ws = machine.Pin(LED_PIN_WS, machine.Pin.OUT)

# SET WEB SOCKET CONNECTION VARIABLES HERE
WEB_SOCKET_ADDRESS = '192.168.10.225'
WEB_SOCKET_PORT = '1880'
WEB_SOCKET_PATH = '/ws'


# ---------- CONFIG ----------
WS_URL = "ws://" + WEB_SOCKET_ADDRESS + ":" + WEB_SOCKET_PORT + WEB_SOCKET_PATH
W5_STATIC = (IP_ADDRESS, SUBNET_MASK, GATEWAY, DNS)

W5_SPI_ID = 0
W5_SCK = 18; W5_MOSI = 19; W5_MISO = 16
W5_CS_PIN = 17; W5_RESET_PIN = 20

UART_ID = 0; UART_TX_PIN = 0; UART_RX_PIN = 1; UART_BAUD = 230400

W5_INIT_TIMEOUT = 8
W5_RECONNECT_TIMEOUT = 2
HEARTBEAT_INTERVAL = 30
WS_CONNECT_TIMEOUT = 6
WS_SEND_TIMEOUT = 5
WS_RECONNECT_DELAY = 3
SEND_QUEUE_MAX = 32         # bounded offline backlog
MAX_FRAME_LEN = 2048        # lower limit for framed message
MAX_READ_CHUNK = 1024      # never read more than this at once from UART
MAX_UART_BUF = 8192        # cap total in-memory UART buffer to ~8 KB
WS_SEND_CHUNK = 512
WS_MAX_CONNECTION_AGE = 600
LINK_CHECK_INTERVAL = 2
FRAME_MAGIC = b'\xa5\x5a'
FRAME_MAGIC_ONLY = True   # set False only if a peer still sends bare len16
WDT_TIMEOUT_MS = 15000

USE_WDT = True

# ---------- helpers ----------
def _rand_bytes(n):
    try:
        import urandom
        return bytes(bytearray(urandom.getrandbits(8) for _ in range(n)))
    except Exception:
        t = int(time.time()*1000) & 0xFFFFFFFF
        out = bytearray()
        while len(out) < n:
            t = (t*1664525 + 1013904223) & 0xFFFFFFFF
            out.append((t>>8)&0xFF)
        return bytes(out)

def _gen_key_b64(): return ubinascii.b2a_base64(_rand_bytes(16)).strip()

class RingQueue:
    def __init__(self, max_items):
        self.items = [None] * max_items
        self.max_items = max_items
        self.head = 0
        self.tail = 0
        self.count = 0
        self.dropped = 0

    def append(self, item):
        if self.count >= self.max_items:
            self.items[self.head] = None
            self.head = (self.head + 1) % self.max_items
            self.count -= 1
            self.dropped += 1

        self.items[self.tail] = item
        self.tail = (self.tail + 1) % self.max_items
        self.count += 1

    def peek(self):
        if self.count == 0:
            return None
        return self.items[self.head]

    def popleft(self):
        if self.count == 0:
            return None

        item = self.items[self.head]
        self.items[self.head] = None
        self.head = (self.head + 1) % self.max_items
        self.count -= 1
        return item

    def clear(self):
        while self.count:
            self.popleft()

def _parse_ws_url(url):
    secure = url.startswith("wss://"); rest = url.split("://",1)[1]
    if "/" in rest: hostport,path = rest.split("/",1); path="/" + path
    else: hostport, path = rest, "/"
    if ":" in hostport: host, port = hostport.split(":",1); port=int(port)
    else: host, port = hostport, (443 if secure else 80)
    return secure, host, port, path

def ws_connect(url, timeout=WS_CONNECT_TIMEOUT):
    secure, host, port, path = _parse_ws_url(url)
    if secure: raise NotImplementedError("wss not supported")
    s = None
    try:
        addr = socket.getaddrinfo(host, port)[0][-1]
        s = socket.socket()
        s.settimeout(timeout)
        s.connect(addr)
        key = _gen_key_b64()
        req = ("GET {} HTTP/1.1\r\nHost: {}:{}\r\nUpgrade: websocket\r\n"
               "Connection: Upgrade\r\nSec-WebSocket-Key: {}\r\nSec-WebSocket-Version: 13\r\n\r\n").format(path, host, port, key.decode())
        s.send(req.encode())
        resp = s.recv(1024)
        if not resp or b"101" not in resp.split(b"\r\n")[0]:
            raise OSError("WS handshake failed")
        s.settimeout(WS_SEND_TIMEOUT)  # never leave socket blocking forever
        return s
    except Exception:
        if s:
            try: s.close()
            except Exception: pass
        raise

def _send_all(sock, data):
    sent_total = 0
    data_len = len(data)
    while sent_total < data_len:
        sent = sock.send(data[sent_total:])
        if sent is None:
            sent = data_len - sent_total
        if sent == 0:
            raise OSError("socket closed")
        sent_total += sent

def ws_send_text(sock, text):
    payload = text.encode()
    fin_opcode = 0x81; n = len(payload)
    header = bytearray([fin_opcode])
    mask_bit = 0x80
    if n < 126:
        header.append(mask_bit | n)
    elif n < (1<<16):
        header.append(mask_bit | 126)
        header += bytearray([(n>>8)&0xff, n&0xff])
    else:
        header.append(mask_bit | 127)
        for shift in (56,48,40,32,24,16,8,0):
            header.append((n>>shift)&0xff)
    mask = _rand_bytes(4)
    header += mask
    sock.settimeout(WS_SEND_TIMEOUT)
    try:
        _send_all(sock, header)
        pos = 0
        masked = bytearray(min(WS_SEND_CHUNK, n if n else 1))
        while pos < n:
            chunk_len = min(len(masked), n - pos)
            for i in range(chunk_len):
                masked[i] = payload[pos + i] ^ mask[(pos + i) % 4]
            _send_all(sock, memoryview(masked)[:chunk_len])
            pos += chunk_len
    finally:
        sock.settimeout(WS_SEND_TIMEOUT)

def ws_close(sock):
    try: sock.close()
    except Exception: pass

def ws_send_ping(sock):
    """RFC6455 ping (opcode 0x9), masked client frame, empty payload."""
    mask = _rand_bytes(4)
    frame = bytearray([0x89, 0x80]) + mask  # FIN+ping, mask bit, len 0
    sock.settimeout(WS_SEND_TIMEOUT)
    _send_all(sock, frame)

def ws_poll_peer(sock):
    """Return False only on WS close opcode or hard socket error.
    True = alive / nothing definitive. Do NOT treat bare empty recv as dead
    (W5500 MicroPython can spuriously return b'' and cause false reconnects).
    Real NR drops are caught by close frames, hard OSError, or heartbeat ping.
    """
    if sock is None:
        return False
    try:
        sock.settimeout(0)
        data = sock.recv(256)
        sock.settimeout(WS_SEND_TIMEOUT)
        if not data:
            # None or b'' with no close opcode — inconclusive on W5500; stay up
            return True
        # Close frame: opcode 0x8 in low nibble of first header byte
        op = data[0] & 0x0f
        if op == 0x8:
            return False
        return True
    except OSError as e:
        sock.settimeout(WS_SEND_TIMEOUT)
        errno = getattr(e, "errno", None)
        # EAGAIN / ETIMEDOUT / EWOULDBLOCK => nothing to read => alive
        if errno in (11, 35, 110, 116):
            return True
        msg = str(e).lower()
        if "timeout" in msg or "eagain" in msg or "would block" in msg:
            return True
        return False  # hard error => dead
    except Exception:
        try:
            sock.settimeout(WS_SEND_TIMEOUT)
        except Exception:
            pass
        return False

# ---------- W5500 init ----------
def _feed_wdt(wdt):
    try:
        if wdt:
            wdt.feed()
    except Exception:
        pass

def w5_init(wdt=None, connect_timeout=W5_INIT_TIMEOUT):
    try:
        import network
        spi = machine.SPI(W5_SPI_ID, baudrate=2_000_000, polarity=0, phase=0,
                          sck=machine.Pin(W5_SCK), mosi=machine.Pin(W5_MOSI), miso=machine.Pin(W5_MISO))
        nic = network.WIZNET5K(spi, machine.Pin(W5_CS_PIN, machine.Pin.OUT), machine.Pin(W5_RESET_PIN, machine.Pin.OUT))
        nic.active(True)
        try:
            if W5_STATIC: nic.ifconfig(W5_STATIC)
        except Exception:
            pass
        start = time.time()
        while not nic.isconnected():
            _feed_wdt(wdt)
            if time.time() - start > connect_timeout:
                return None
            time.sleep(0.25)
        return nic
    except Exception:
        return None

def link_is_up(nic):
    try:
        return bool(nic and nic.isconnected())
    except Exception:
        return False

# ---------- UART & parsing ----------
uart = machine.UART(UART_ID, UART_BAUD, tx=machine.Pin(UART_TX_PIN), rx=machine.Pin(UART_RX_PIN), timeout=10)
uart_buf = b""

def _extract_brace_json(buf):
    s = buf.find(b'{')
    if s == -1: return None, buf
    e = buf.find(b'}', s+1)
    if e == -1: return None, buf
    part = buf[s:e+1]
    try:
        return part.decode('utf-8'), buf[e+1:]
    except Exception:
        return None, buf

def _extract_line(buf):
    i = buf.find(b'\n')
    if i == -1: return None, buf
    line = buf[:i].rstrip(b'\r')
    try:
        return line.decode('utf-8'), buf[i+1:]
    except Exception:
        return None, buf

def _decode_payload(payload):
    try:
        return payload.decode('utf-8')
    except Exception:
        return payload.decode('utf-8', 'ignore')

def _extract_uart_message(buf):
    if not buf:
        return None, buf, 0

    if len(buf) >= 2:
        magic_at = buf.find(FRAME_MAGIC)
        if magic_at > 0:
            return None, buf[magic_at:], 1

        if magic_at == 0:
            if len(buf) < 4:
                return None, buf, 0

            frame_len = (buf[2] << 8) | buf[3]
            if frame_len == 0 or frame_len > MAX_FRAME_LEN:
                return None, buf[1:], 1
            if len(buf) < 4 + frame_len:
                return None, buf, 0

            payload = buf[4:4+frame_len]
            return _decode_payload(payload), buf[4+frame_len:], 0

    if not FRAME_MAGIC_ONLY and len(buf) >= 2:
        frame_len = (buf[0] << 8) | buf[1]
        if 0 < frame_len <= MAX_FRAME_LEN and len(buf) >= 2 + frame_len:
            payload = buf[2:2+frame_len]
            return _decode_payload(payload), buf[2+frame_len:], 0

    if not FRAME_MAGIC_ONLY:
        text, new_buf = _extract_brace_json(buf)
        if text is not None:
            return text, new_buf, 0

        text, new_buf = _extract_line(buf)
        if text is not None:
            return text, new_buf, 0

    if len(buf) > MAX_UART_BUF:
        return None, buf[-MAX_FRAME_LEN:], 1

    return None, buf, 0

def flash_led():
    led_ws.on()
    time.sleep(0.01)
    led_ws.off()
    
# ---------- main loop ----------
def main():
    global uart_buf
    # WDT
    wdt = None
    if USE_WDT:
        try: wdt = machine.WDT(timeout=WDT_TIMEOUT_MS)
        except Exception: wdt = None

    nic = w5_init(wdt, W5_INIT_TIMEOUT)
    print("Network Connection Info")
    try:
        print(nic.ifconfig() if nic else "W5500 not connected")
    except Exception:
        print("W5500 not connected")
    ws = None
    print(WS_URL)
    send_queue = RingQueue(SEND_QUEUE_MAX)
    last_ws_attempt = 0
    last_hb = time.time()
    last_link_check = 0
    last_rx = 0
    last_sent = 0
    ws_connected_at = 0
    rx_bytes = 0
    frames_parsed = 0
    ws_sent = 0
    ws_send_errors = 0
    bad_frames = 0
    parse_errors = 0
    ws_reconnects = 0
    link_drops = 0

    # initial connect attempt
    start = time.time()
    while ws is None and time.time() - start < 10:
        _feed_wdt(wdt)
        if nic is None:
            nic = w5_init(wdt, W5_RECONNECT_TIMEOUT)
        if nic is None:
            time.sleep(1)
            continue
        try:
            ws = ws_connect(WS_URL)
            ws_connected_at = time.time()
            ws_reconnects += 1
            print("WS connected")
        except Exception:
            _feed_wdt(wdt)
            time.sleep(1)

    while True:
        try:
            if wdt: wdt.feed()
        except Exception:
            pass

        if time.time() - last_link_check >= LINK_CHECK_INTERVAL:
            last_link_check = time.time()
            if not link_is_up(nic):
                link_drops += 1
                if ws:
                    print("ETH link down; closing WebSocket")
                    try: ws_close(ws)
                    except Exception: pass
                    ws = None
                nic = None
            elif ws:
                # Detect Node-RED / peer closing the WS while we are idle
                if not ws_poll_peer(ws):
                    print("WS: peer closed or dead; reconnecting")
                    try: ws_close(ws)
                    except Exception: pass
                    ws = None

        # read at most MAX_READ_CHUNK bytes to avoid large allocs
        try:
            avail = uart.any()
            if avail:
                to_read = avail if avail <= MAX_READ_CHUNK else MAX_READ_CHUNK
                chunk = uart.read(to_read) or b''
                rx_bytes += len(chunk)
                last_rx = time.time()
                uart_buf += chunk

                # enforce hard cap on uart_buf to avoid runaway growth
                if len(uart_buf) > MAX_UART_BUF:
                    # drop the oldest bytes; keep tail
                    uart_buf = uart_buf[-MAX_UART_BUF:]
                    gc.collect()

                # parsing loop (magic-framed, legacy-framed, then text fallback)
                while True:
                    text, new_buf, bad = _extract_uart_message(uart_buf)
                    bad_frames += bad
                    uart_buf = new_buf
                    if text is None:
                        break

                    frames_parsed += 1
                    flash_led()

                    # limit text size (defensive)
                    if len(text) > MAX_FRAME_LEN:
                        parse_errors += 1
                        uart_buf = b''
                        gc.collect()
                        break

                    # build envelope (try/catch)
                    obj = None
                    try:
                        obj = json.loads(text)
                        out_text = json.dumps({"payload": obj, "rcv_ts": time.time()})
                    except Exception:
                        parse_errors += 1
                        out_text = json.dumps({"raw": text, "rcv_ts": time.time()})

                    if ws:
                        try:
                            ws_send_text(ws, out_text)
                            ws_sent += 1
                            last_sent = time.time()
                        except MemoryError:
                            ws_send_errors += 1
                            send_queue.clear()
                            try: ws_close(ws)
                            except Exception: pass
                            ws = None
                            gc.collect()
                        except Exception:
                            ws_send_errors += 1
                            try: ws_close(ws)
                            except Exception: pass
                            ws = None
                            send_queue.append(out_text)
                    else:
                        send_queue.append(out_text)

                    # ack back if seq present
                    try:
                        if obj and "seq" in obj:
                            ack_line = "ACK:{}\n".format(int(obj["seq"])); uart.write(ack_line.encode('utf-8'))
                    except Exception:
                        pass
            else:
                time.sleep(0.01)

        except MemoryError:
            # attempt some recovery: drop buffers, collect, close ws
            uart_buf = b''
            send_queue.clear()
            gc.collect()
            try:
                if ws:
                    ws_close(ws); ws = None
            except Exception:
                pass
            time.sleep(0.5)
            continue
        except Exception:
            # generic protection - don't crash on unexpected error
            gc.collect()

        # flush queue
        if ws:
            try:
                while send_queue.count:
                    _feed_wdt(wdt)
                    pkt = send_queue.peek()
                    try:
                        ws_send_text(ws, pkt)
                        ws_sent += 1
                        last_sent = time.time()
                        send_queue.popleft()
                    except Exception:
                        ws_send_errors += 1
                        try: ws_close(ws)
                        except: pass
                        ws = None
                        break
            except Exception:
                gc.collect()

        # Periodically recycle the WebSocket to avoid stale long-lived sockets.
        if ws and ws_connected_at and (time.time() - ws_connected_at) > WS_MAX_CONNECTION_AGE:
            print("WS: scheduled reconnect")
            try: ws_close(ws)
            except Exception: pass
            ws = None

        # background reconnect
        if ws is None and (time.time() - last_ws_attempt) > WS_RECONNECT_DELAY:
            last_ws_attempt = time.time()
            if not link_is_up(nic):
                nic = None
            if nic is None:
                nic = w5_init(wdt, W5_RECONNECT_TIMEOUT)
            if nic:
                try:
                    ws = ws_connect(WS_URL)
                    ws_connected_at = time.time()
                    ws_reconnects += 1
                    print("WS connected")
                except Exception:
                    ws = None

        # heartbeat
        if time.time() - last_hb >= HEARTBEAT_INTERVAL:
            last_hb = time.time()
            if ws:
                try:
                    ws_send_ping(ws)
                except Exception:
                    print("WS: ping failed; reconnecting")
                    try: ws_close(ws)
                    except Exception: pass
                    ws = None
            try:
                rx_age = int(time.time() - last_rx) if last_rx else -1
                sent_age = int(time.time() - last_sent) if last_sent else -1
                print("HEARTBEAT: free mem:", gc.mem_free(),
                      "buf:", len(uart_buf),
                      "rxB:", rx_bytes,
                      "frames:", frames_parsed,
                      "sent:", ws_sent,
                      "send_err:", ws_send_errors,
                      "bad:", bad_frames,
                      "parse_err:", parse_errors,
                      "q:", send_queue.count,
                      "dropped:", send_queue.dropped,
                      "rx_age:", rx_age,
                      "sent_age:", sent_age,
                      "reconn:", ws_reconnects,
                      "link_drops:", link_drops,
                      "ws:", "up" if ws else "down")
            except Exception:
                pass

if __name__ == "__main__":
    main()

