# Lora to Ethernet Bridge

Dual-board MicroPython bridge that receives RadioHead-style LoRa packets on an RFM95, forwards them over UART, and publishes them to a Node-RED WebSocket over Ethernet.

A client that sends temperature readings is [Local LoRa Temperature Probe Sensor](https://github.com/scottlecomte/Local-LoRa-Temperature-Probe-Sensor).

```
RFM95 → Raspberry Pi Pico (LoRa RX) → UART @ 230400 → W5500-EVB-Pico → Ethernet → Node-RED ws://…/ws
```

## Hardware

### Board A — LoRa Pico

| Item | Notes |
|------|--------|
| MCU | Raspberry Pi Pico (RP2040), MicroPython |
| Radio | HopeRF / Adafruit-style **RFM95** (915 MHz in this tree) |
| Role | LoRa receive → framed UART TX; RadioHead ACKs for packets addressed to this server |

**RFM95 wiring (this firmware)**

| RFM95 | Pico GPIO | Notes |
|-------|-----------|--------|
| NSS / CS | GP8 | `RFM95_CS` |
| RESET | GP9 | `RFM95_RST` |
| DIO0 / INT | GP22 | `RFM95_INT` |
| SCK | GP18 | SPI0 (`SPIConfig.rp2_0`) |
| MOSI | GP19 | SPI0 |
| MISO | GP16 | SPI0 |
| VIN / 3V3 | 3V3 | Do not feed 5 V into RFM95 IO |
| GND | GND | Common ground with Wiznet board |

**UART to Wiznet (cross-over)**

| LoRa Pico | Wiznet Pico |
|-----------|-------------|
| GP0 TX | GP1 RX |
| GP1 RX | GP0 TX |
| GND | GND |

Baud: **230400**, UART0 both sides.

Optional LEDs (LoRa board): GP12 = LoRa RX activity, GP13 = UART TX activity.

Radio address: this node is RadioHead **server address 2** (`SERVER_ADDRESS = 2`). `receive_all=True` so other destinations are heard, but only packets with `header_to == SERVER_ADDRESS` get RadioHead ACKs (`acks=True`).

### Board B — Wiznet Pico

| Item | Notes |
|------|--------|
| Board | **WIZnet W5500-EVB-Pico** (RP2040 + W5500 on-board) |
| Role | UART RX → Ethernet WebSocket client → Node-RED |

On-board W5500 SPI (as used in firmware; matches EVB defaults):

| Signal | GPIO |
|--------|------|
| SCK | GP18 |
| MOSI | GP19 |
| MISO | GP16 |
| CS | GP17 |
| RESET | GP20 |

Static Ethernet (edit in `wiznet-pico/main.py`):

| Setting | Example in tree |
|---------|-----------------|
| IP | `xxx.xxx.xxx.xxx` |
| Subnet | `xxx.xxx.xxx.xxx` |
| Gateway / DNS | `xxx.xxx.xxx.xxx` |
| WebSocket | `ws://xxx.xxx.xxx.xxx:1880/ws` |

## UART framing

Both ends use:

```
A5 5A | len_hi | len_lo | payload…
```

- Magic: `0xA5 0x5A`
- Length: big-endian uint16 of **payload** bytes only
- Payload: UTF-8 JSON from the LoRa side, e.g. `message`, `header_*`, `rssi`, `snr`, `ts`

Wiznet wraps each decoded message as `{"payload": <obj>, "rcv_ts": <epoch>}` on the WebSocket.

`FRAME_MAGIC_ONLY = True` disables legacy bare-length and brace/line fallbacks — flash **both** boards when changing framing.

## Firmware layout

```
lora-pico/
  main.py          # LoRa RX → framed UART; drains deferred RadioHead ACKs in main loop
  lib/ulora.py     # RFM95 / RadioHead-compatible driver (deferred ACK queue + TX_DONE poll)
wiznet-pico/
  main.py          # UART → W5500 WebSocket + reconnect / WDT
```

Flash each `main.py` (and `ulora` under `lib/` on the LoRa Pico) with Thonny / `mpremote` as MicroPython.

## RadioHead ACKs

- **`acks=True`**: clients that use RadioHead-style `send_to_wait` get a short ACK (`b'!'`, `FLAGS_ACK`) when `header_to == SERVER_ADDRESS`. Packets for other destinations are still received (`receive_all=True`) and do not get an ACK.
- **Forward anyway**: the bridge does not wait on that ACK. Every received frame is still framed onto UART and published over Ethernet whether or not the RF ACK goes out.
- **Deferred ACK**: the RX IRQ only queues `(header_from, header_id)` on `_pending_acks`. `process_pending_acks()` runs from the main loop — never `send` / `wait_packet_sent` inside the RX IRQ.
- **TX_DONE / RX_DONE**: DIO0 is unreliable. `wait_packet_sent()` polls `TX_DONE` and the op-mode register so a missed TxDone edge does not burn the ACK window. RX completion is the `RX_DONE` flag in the DIO handler; those IRQ flags are cleared immediately so DIO0 can re-arm.
- **CDC / Thonny**: do not leave Thonny stopped on the Pico’s USB CDC. A stopped REPL holds the MCU, so the main loop does not run — no UART forward, and no ACKs. Disconnect or let the script run after flash.

## Reliability notes (2026-09)

- LoRa IRQ path stays non-blocking (`micropython.schedule`); LED pulse runs in the main loop; RX queue capped at 32; ACK drain runs in the main loop (see above).
- Wiznet keeps finite socket timeouts, polls for WebSocket close, sends a WS ping on the 30 s heartbeat, and reconnects when Node-RED drops the socket on flow deploy.
- Wiznet WDT timeout 15 s (`USE_WDT = True`).

## Node-RED

Point a WebSocket **server** node at path `/ws` (or change `WEB_SOCKET_PATH`). Parse the text as JSON; client payload fields are under `payload`.


## License / origin

Personal home-automation firmware. Adapt pin maps and IPs for your site before deploying.
