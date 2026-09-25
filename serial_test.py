"""Serial link to the Arduino motor controller.

Commands (one ASCII byte each):

    S  stop          F  forward
    L  turn left     R  turn right

The port is taken from the ARDUINO_PORT environment variable, or detected
automatically. If no board is connected the module runs in dry-run mode and
logs the commands instead, so the vision scripts can be tested on a laptop.
"""

import logging
import os
import threading

log = logging.getLogger("arduino")

COMMANDS = {"S": "stop", "F": "forward", "L": "left", "R": "right"}
BAUDRATE = int(os.environ.get("ARDUINO_BAUD", "9600"))

_lock = threading.Lock()
_serial = None
_connected = False
_last_sent = None


def _find_port():
    port = os.environ.get("ARDUINO_PORT")
    if port:
        return port
    try:
        from serial.tools import list_ports
    except ImportError:
        return None
    for p in list_ports.comports():
        desc = f"{p.description} {p.manufacturer or ''}".lower()
        if "arduino" in desc or "ch340" in desc or "usb serial" in desc:
            return p.device
    return None


def _connect():
    global _serial, _connected
    _connected = True  # only try once
    port = _find_port()
    if not port:
        log.warning("No Arduino found; running in dry-run mode (set ARDUINO_PORT to override).")
        return
    try:
        import serial

        _serial = serial.Serial(
            port, baudrate=BAUDRATE, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1, write_timeout=1,
        )
        log.info("Connected to Arduino on %s at %d baud", port, BAUDRATE)
    except Exception as exc:  # pyserial raises SerialException, OSError, ValueError
        log.warning("Could not open %s (%s); running in dry-run mode.", port, exc)
        _serial = None


def Send(msg: str, *, force: bool = False) -> bool:
    """Send a command byte. Repeats of the last command are skipped unless ``force``.

    Returns True if the command was written to the board.
    """
    global _last_sent
    if msg not in COMMANDS:
        raise ValueError(f"Unknown command {msg!r}; expected one of {sorted(COMMANDS)}")
    with _lock:
        if msg == _last_sent and not force:
            return False
        if not _connected:
            _connect()
        _last_sent = msg
        if _serial is None:
            log.info("[dry-run] %s (%s)", msg, COMMANDS[msg])
            return False
        try:
            _serial.write(msg.encode("ascii"))
            return True
        except Exception as exc:
            log.error("Serial write failed: %s", exc)
            return False


def close() -> None:
    global _serial
    with _lock:
        if _serial is not None:
            try:
                _serial.write(b"S")  # leave the car stopped
            finally:
                _serial.close()
                _serial = None
