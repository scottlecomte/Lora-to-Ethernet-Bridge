# lora_to_uart_fire_and_forget.py
# Simple LoRa -> UART forwarder (no ACK wait). Uses UART0 (GP0 TX, GP1 RX).

import time, machine, ubinascii, ujson as json, micropython
from ulora import LoRa, SPIConfig

# config
RFM95_RST = 9; RFM95_CS = 8; RFM95_INT = 22
RF95_FREQ = 915.0; RF95_POW = 20; SERVER_ADDRESS = 2
UART_ID = 0; UART_TX_PIN = 0; UART_RX_PIN = 1; UART_BAUD = 230400

# LED Setup
LED_PIN_UART = 13
LED_PIN_LORA = 12
led_uart = machine.Pin(LED_PIN_UART, machine.Pin.OUT)
led_lora = machine.Pin(LED_PIN_LORA, machine.Pin.OUT)

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
uart = machine.UART(UART_ID, UART_BAUD, tx=machine.Pin(UART_TX_PIN), rx=machine.Pin(UART_RX_PIN), timeout=10)

def _enqueue_payload(payload):
    try:
        if len(_payload_queue) >= _QUEUE_MAX:
            _payload_queue.pop(0)  # drop oldest under burst
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

def _frame_and_send(payload_bytes):
    L = len(payload_bytes)
    # Match Wiznet FRAME_MAGIC path: A5 5A + uint16_be length + body
    hdr = b'\xa5\x5a' + bytes([(L>>8) & 0xFF, L & 0xFF])
    try:
        uart.write(hdr + payload_bytes)
        print("FF: wrote frame len", len(payload_bytes))
        led_uart.on()
        time.sleep(0.01)
        led_uart.off()
    except Exception as e:
        print("FF: write error", e)

def run():
    lora = LoRa(SPIConfig.rp2_0, RFM95_INT, SERVER_ADDRESS, RFM95_CS,
                reset_pin=RFM95_RST, freq=RF95_FREQ, tx_power=RF95_POW, receive_all=True, acks=True)
    lora.on_recv = on_recv_callback
    try:
        lora.set_mode_rx()
    except Exception:
        pass

    print("FF forwarder running UART {} @ {} acks={}".format(UART_ID, UART_BAUD, getattr(lora, "_acks", None)))
    try:
        while True:
            # Drain RF ACKs outside IRQ (ulora queues them on RX) — before UART work
            try:
                lora.process_pending_acks()
            except Exception as e:
                print("ACK drain err", e)
            global _led_lora_pending
            if _led_lora_pending:
                _led_lora_pending = False
                led_lora.on()
                time.sleep(0.01)
                led_lora.off()
            if _payload_queue:
                payload = _payload_queue.pop(0)
                if _drop_uart_dup(payload):
                    # Later copy, same header_from + header_id. First copy already went out.
                    print("FF: drop dup from", int(payload.header_from), "id", int(payload.header_id))
                else:
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
                    print("ACK drain err", e)
            else:
                time.sleep(0.01)
    finally:
        try:
            lora.close()
        except:
            pass

if __name__ == "__main__":
    run()
