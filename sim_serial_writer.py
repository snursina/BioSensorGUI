# sim_serial_writer.py
import sys, time, math, random, serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM12"
BAUD = 115200

HELLO_LINE   = b"SIM:HELLO v1\n"
ACK_PREFIX   = "SIM:ACK"
START_PREFIX = "SIM:START"
STOP_PREFIX  = "SIM:STOP"

HELLO_INTERVAL = 0.5   # seconds between HELLO beacons
STREAM_DT      = 0.04  # seconds between data points (~25 Hz)

def gen_point(t, r0=100.0, r_arc=50.0, f0=1000.0):
    th = (t*0.8) % math.pi
    re = r0 + r_arc*math.cos(th) + random.uniform(-0.3, 0.3)
    im = r_arc*math.sin(th) + random.uniform(-0.3, 0.3)
    freq = f0 * (1.5 + 0.5*math.sin(t*0.2))
    return re, im, freq

def _read_line(ser):
    raw = ser.readline()
    if not raw:
        return None
    try:
        return raw.decode("ascii", errors="ignore").strip()
    except Exception:
        return None

def advertise_until_ack(ser):
    """Keep sending SIM:HELLO until we read SIM:ACK."""
    last_hello = 0.0
    while True:
        now = time.time()
        if now - last_hello >= HELLO_INTERVAL:
            ser.write(HELLO_LINE)
            last_hello = now
        line = _read_line(ser)
        if not line:
            continue
        if line.startswith(ACK_PREFIX):
            return  # got ACK, next wait for START

def wait_for_start(ser, timeout=None):
    """Wait for SIM:START (optionally with a timeout)."""
    deadline = (time.time() + timeout) if timeout else None
    while True:
        if deadline and time.time() >= deadline:
            return False
        line = _read_line(ser)
        if not line:
            continue
        if line.startswith(START_PREFIX):
            return True

def stream_until_stop(ser):
    """Stream CSV 'real,imag,freq' lines until SIM:STOP is received."""
    t0 = time.time()
    next_tick = time.time()
    while True:
        # Check for STOP quickly (non-blocking-ish because of short timeout)
        line = _read_line(ser)
        if line and line.startswith(STOP_PREFIX):
            return
        # Emit a point at fixed rate
        now = time.time()
        if now >= next_tick:
            t = now - t0
            re, im, f = gen_point(t)
            ser.write(f"{re:.6f},{im:.6f}, {f:.2f}\n".encode("ascii"))
            next_tick = now + STREAM_DT
        else:
            # small sleep to avoid busy loop
            time.sleep(0.005)

def main():
    # Small read timeout so readline() returns quickly
    with serial.Serial(PORT, BAUD, timeout=0.05) as ser:
        while True:
            # Phase A: advertise until the app acknowledges
            advertise_until_ack(ser)
            # Phase B: wait for START (your app sends ACK then START on "Start Measurement")
            if not wait_for_start(ser, timeout=5.0):
                # If the app never sent START, go back to HELLO and try again
                continue
            # Phase C: stream until STOP, then loop back to HELLO automatically
            stream_until_stop(ser)
            # Now we loop and start advertising again for the next measurement

if __name__ == "__main__":
    main()
