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
                reset_pin=RFM95_RST, freq=RF95_FREQ, tx_power=RF95_POW, receive_all=True, acks=False)
    lora.on_recv = on_recv_callback
    try:
        lora.set_mode_rx()
    except Exception:
        pass

    print("FF forwarder running UART {} @ {}".format(UART_ID, UART_BAUD))
    try:
        while True:
            global _led_lora_pending
            if _led_lora_pending:
                _led_lora_pending = False
                led_lora.on()
                time.sleep(0.01)
                led_lora.off()
            if _payload_queue:
                payload = _payload_queue.pop(0)
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
            else:
                time.sleep(0.02)
    finally:
        try:
            lora.close()
        except:
            pass

if __name__ == "__main__":
    run()
