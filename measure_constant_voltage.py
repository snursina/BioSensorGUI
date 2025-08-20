#!/usr/bin/env python3
import time, struct, serial, numpy as np, matplotlib.pyplot as plt

# -----------------------------
# USER SETTINGS
# -----------------------------
PORT             = "COM5"
BAUD             = 115200
STEP_VOLTAGE_MV  = 0      # constant step in mV
RTIA_OHMS        = 10000
DURATION_SEC     = 5       # run duration (None = infinite until Ctrl+C)
SAMPLES_PER_SEC  = 1000     # for time axis scaling
SAVE_FILE        = None#"const_voltage_run.csv"

# -----------------------------
# Command IDs (must match FW!)
# -----------------------------
CMD_START_CONFIG      = 5
CMD_STOP_CONFIG       = 6
CMD_SET_VOLTAGE_STEP  = 100   # PARAM_VOLTAGE_STEP
CMD_SET_RTIA_OHMS     = 109   # PARAM_RTIA_OHMS
CMD_START_MEAS_CONST  = 1
CMD_STOP_MEAS         = 0
DATA_MEAS_CONST       = 1     # device prints like "1:32760"

# -----------------------------
# ADC → Current conversion
# -----------------------------
SLOPE_TABLE = {
    680:    0.2222,    # µA/code
    1000:   0.1538,
    2000:   0.07663,
    3300:   0.117,
    7500:   0.09756,
    10000:  0.07407,
    20000:  0.07843,
    40000:  -0.01255,
    100000: -0.05634,
    200000: 0.01003,
    512000: 0.01442,
}
INTERCEPT_TABLE = {
    680:    -4386.12,    # µA/code
    1000:   -2982.1,
    2000:   -1402.46,
    3300:   -2230.52,
    7500:   -1839.08,
    10000:  -1355.814,
    20000:  -1441.8,
    40000:  260.03,
    100000: 1104.59,
    200000: -180.22,
    512000: -263.86,
}
SLOPE_UA_PER_CODE = SLOPE_TABLE[RTIA_OHMS] 
CODE_ZERO_REF = 12288 
INTERCEPT_UA = INTERCEPT_TABLE[RTIA_OHMS] 

def code_to_microamps(code: int) -> float:
    return INTERCEPT_UA + (code - CODE_ZERO_REF) * SLOPE_UA_PER_CODE


# -----------------------------
# Helpers
# -----------------------------
class LowPassFilter:
    def __init__(self, factor: int = 8):
        self.factor = factor
        self.filtered_value = None

    def filter(self, new_sample: float) -> float:
        if self.filtered_value is None:
            self.filtered_value = new_sample * self.factor
        else:
            self.filtered_value = (self.filtered_value -
                                   (self.filtered_value / self.factor)) + new_sample
        return self.filtered_value / self.factor
    
    def clear(self):
        self.filtered_value = None

def send_cmd(ser, cmd_id, int_value=None):
    ser.write(bytes([cmd_id]))
    if int_value is not None:
        ser.write(struct.pack("<i", int_value))

def read_ack(ser, timeout=1.0):
    ser.timeout = timeout
    b = ser.read(1)
    return b[0] if len(b) == 1 else None

def parse_line(line: str):
    try:
        parts = line.strip().split(":")
        if len(parts) != 2: return None, None
        ident, val = int(parts[0]), int(parts[1])
        return ident, val
    except:
        return None, None

# -----------------------------
# Main routine
# -----------------------------
def main():
    times, currents = [], []
    dt = 1.0 / SAMPLES_PER_SEC
    sample_idx = 0

    with serial.Serial(PORT, BAUD, timeout=0.05) as ser:
        ser.reset_input_buffer()

        # --- CONFIGURATION ---
        send_cmd(ser, CMD_START_CONFIG)
        print("[cfg] START_CONFIG")

        send_cmd(ser, CMD_SET_VOLTAGE_STEP, STEP_VOLTAGE_MV * 1000)  # mV → µV
        print(f"[cfg] SET_VOLTAGE_STEP={STEP_VOLTAGE_MV} mV")

        send_cmd(ser, CMD_SET_RTIA_OHMS, RTIA_OHMS)
        print(f"[cfg] SET_RTIA_OHMS={RTIA_OHMS} Ω")

        send_cmd(ser, CMD_STOP_CONFIG)
        print("[cfg] STOP_CONFIG")

        # --- START MEAS ---
        send_cmd(ser, CMD_START_MEAS_CONST)
        print("[run] START_MEAS_CONST sent")

        t0 = time.time()
        plt.ion()
        fig, ax = plt.subplots()
        lineplot, = ax.plot([], [])
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Current [µA]")

        uA_filter = LowPassFilter(factor=8)
        try:
            while True:
                if ser.in_waiting:
                    raw = ser.readline().decode(errors="ignore").strip()
                    mid, code = parse_line(raw)
                    if mid == DATA_MEAS_CONST:
                        t_rel = sample_idx * dt
                        print(code)
                        i_uA = uA_filter.filter(code_to_microamps(code))
                        times.append(t_rel)
                        currents.append(i_uA)
                        sample_idx += 1

                        if sample_idx % 50 == 0:
                            lineplot.set_data(times, currents)
                            ax.relim(); ax.autoscale_view()
                            plt.pause(0.001)

                if DURATION_SEC and (time.time() - t0) > DURATION_SEC:
                    break
        finally:
            send_cmd(ser, CMD_STOP_MEAS)
            print("[run] STOP_MEAS sent")

    # --- SAVE ---
    if SAVE_FILE:
        np.savetxt(SAVE_FILE, np.column_stack((times, currents)),
                delimiter=",", header="time_s,current_uA", comments="")
        print(f"[save] wrote {SAVE_FILE} (samples={len(times)})")

    plt.ioff(); plt.show()

if __name__ == "__main__":
    main()
