import serial
import time
import struct
import numpy as np
#import threading
import matplotlib.pyplot as plt
import pylink
import math

MAKE_MEASUREMENT = False # only plots the measurement_log if false

CMD = {
    "START_CONFIG": 5,
    "STOP_CONFIG": 6,
    "SET_VOLTAGE_EIS": 104,
    "SET_NUM_FREQ_EIS": 107,
    "START_MEAS_EIS": 3,
    "STOP_MEAS": 0,
}

def reset_via_pylink(device="aducm350"):
    jlink = pylink.JLink()
    jlink.open()
    jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
    jlink.connect(device)
    jlink.reset()
    jlink.close()

def float_to_microvolts(voltage_mV):
    """Convert mV float to integer microvolts."""
    return int(voltage_mV * 1000 + 0.5)

def send_command(ser, cmd_id, int_value=None):
    """Send a command byte and optional 4-byte integer payload."""
    ser.write(bytes([cmd_id]))
    if int_value is not None:
        ser.write(struct.pack('<i', int_value))  # Little-endian 4-byte int

def send_frequency_array(ser, freq_array):
    """Send frequency array as sequence of 4-byte ints."""
    for freq in freq_array:
        ser.write(struct.pack('<i', freq))

def config_eis(ser, voltage_mV, start_freq, stop_freq, num_points, log_spacing=True):
    log = []
    
    def log_and_print(msg):
        print(msg)
        log.append(msg)

    # Flush serial input buffer
    ser.reset_input_buffer()

    # 1. Start config
    send_command(ser, CMD["START_CONFIG"])
    log_and_print("Started parameter config")

    # 2. Set voltage
    voltage_uV = float_to_microvolts(voltage_mV)
    send_command(ser, CMD["SET_VOLTAGE_EIS"], voltage_uV)
    log_and_print(f"Set voltage to {voltage_mV} mV")

    # 3. Set number of frequency points
    send_command(ser, CMD["SET_NUM_FREQ_EIS"], num_points)

    # 4. Generate frequency array
    if log_spacing:
        log_and_print("Using logarithmic frequency spacing")
        freqs = np.logspace(np.log10(start_freq), np.log10(stop_freq), num_points, dtype=int)
    else:
        log_and_print("Using linear frequency spacing")
        freqs = np.linspace(start_freq, stop_freq, num_points, dtype=int)

    send_frequency_array(ser, freqs)
    log_and_print(f"Sent {num_points} frequency points: {start_freq}–{stop_freq} Hz")

    # 5. Stop config
    send_command(ser, CMD["STOP_CONFIG"])
    log_and_print("Stopped parameter config")

    # 6. Save log
    with open("config_log.txt", "w") as f:
        f.write("\n".join(log))

    print("Configuration complete, log saved to config_log.txt")

# def start_measurement(ser, duration_sec=None):
#     print("\nStarting EIS measurement...")
#     ser.reset_input_buffer()
#     send_command(ser, CMD["START_MEAS_EIS"])

#     with open("measurement_log.txt", "w") as f:
#         start_time = time.time()
#         estimated_toa = None
#         timeout_stop = start_time
#         print("Logging to measurement_log.txt. Press Enter to stop early.\n")
#         stop_flag = threading.Event()

#         def wait_for_enter():
#             input()
#             stop_flag.set()

#         threading.Thread(target=wait_for_enter, daemon=True).start()

#         while not stop_flag.is_set():
#             if ser.in_waiting:
#                 line = ser.readline().decode(errors="ignore").strip()
#                 if line:
#                     print(">>", line)
#                     if estimated_toa is not None: f.write(line + "\n")
#                     timeout_stop = time.time()
#                     if estimated_toa is None: estimated_toa = 1.5 * (time.time() - start_time)
#             if duration_sec and (time.time() - timeout_stop) > duration_sec:
#                 break
#             if estimated_toa and duration_sec is None:    
#                 if (time.time() - timeout_stop) > estimated_toa:
#                     break

#         send_command(ser, CMD["STOP_MEAS"])
#         print("Measurement stopped.")

def start_measurement(ser, duration_sec=None, line_callback=None, stop_event=None):
    """
    Run measurement, optionally call line_callback(line) on each received line.
    If stop_event is provided, it can be used to stop the measurement early.
    """
    print("\nStarting EIS measurement...")
    ser.reset_input_buffer()
    send_command(ser, CMD["START_MEAS_EIS"])

    #with open("measurement_log.txt", "w") as f:
    start_time = time.time()
    estimated_toa = None
    timeout_stop = start_time

    while True:
        if stop_event is not None and stop_event.is_set():
            break
        if ser.in_waiting:
            line = ser.readline().decode(errors="ignore").strip()
            if line:
                if line_callback:
                    line_callback(line)
                    #f.write(line + "\n")
                    #print(">>", line)
                # parsed = parse_line_calibrated(line, calibration_resistor=10000)
                # if parsed:
                #     freq, Zx_re, Zx_im, Zx_mag, Zx_phase = parsed
                #     # Get original Rcal and Zm components
                #     parts = line.strip().split(':')
                #     if len(parts) >= 6:
                #         freq = parts[1]
                #         Rcal_re = parts[2]
                #         Rcal_im = parts[3]
                #         Zm_re = parts[4]
                #         Zm_im = parts[5]
                #         formatted_line = f"3:{freq}:{Rcal_re}:{Rcal_im}:{Zm_re}:{Zm_im}"
                #         f.write(formatted_line + "\n")
                timeout_stop = time.time()
                if estimated_toa is None:
                    estimated_toa = 1.5 * (time.time() - start_time)
        if duration_sec and (time.time() - timeout_stop) > duration_sec:
            break
        if estimated_toa and duration_sec is None:
            if (time.time() - timeout_stop) > estimated_toa:
                break

    send_command(ser, CMD["STOP_MEAS"])
    print("Measurement stopped.")

def parse_line_calibrated(line, calibration_resistor):
    parts = line.strip().split(':')
    # Expecting at least 5 values: e.g. "3:freq:Rcal_re:Rcal_im:Zm_re:Zm_im"
    if len(parts) < 6 or parts[0] != '3':
        return None
    
    
    
    freq = int(parts[1])
    Rcal_re = int(parts[2])
    Rcal_im = int(parts[3])
    Zm_re = int(parts[4])
    Zm_im = int(parts[5])

    #print(f"Rcal: ({Rcal_re}, {Rcal_im}) | Zm: ({Zm_re}, {Zm_im})")

    # Magnitude and phase of Rcal
    Rcal_mag = math.sqrt(Rcal_re**2 + Rcal_im**2)
    Rcal_phase = math.atan2(Rcal_im, Rcal_re)

    # Magnitude and phase of Zm (measured impedance)
    Zm_mag = math.sqrt(Zm_re**2 + Zm_im**2)
    Zm_phase = math.atan2(Zm_im, Zm_re)

    # Handle divide by zero
    if Zm_mag == 0:
        Zx_mag = 0
    else:
        # Calculate magnitude of unknown impedance using calibration resistor
        Zx_mag = (Rcal_mag / Zm_mag) * calibration_resistor
    
    # Phase difference
    Zx_phase = Zm_phase - Rcal_phase

    # Calculate real and imaginary parts of unknown impedance Zx
    Zx_re = round(Zx_mag * math.cos(Zx_phase), 2)
    Zx_im = round(Zx_mag * math.sin(Zx_phase), 2)

    # Adjust imaginary part sign to match plotting conventions (optional)
    Zx_im = -Zx_im

    return freq, Zx_re/10, Zx_im/10, Zx_mag/10, math.degrees(Zx_phase)

def set_min_lim(y_data, axis=1, min_range=1000, ax=None):
    """
    Sets y-axis limits with at least `min_range` total span.
    
    Parameters:
        y_data (list or array): The data to determine current y-limits.
        min_range (float): The minimum total range for the y-axis.
        ax (matplotlib.axes.Axes, optional): An existing Axes object. If None, uses plt.gca().
    """
    if ax is None:
        ax = plt.gca()
    
    if not y_data:
        return  # Avoid errors on empty data

    y_min = min(y_data)
    y_max = max(y_data)
    actual_range = y_max - y_min

    if actual_range < min_range:
        center = (y_max + y_min) / 2
        half_range = min_range / 2
        if axis==1: ax.set_ylim(center - half_range, center + half_range)
        elif axis==0: ax.set_xlim(center - half_range, center + half_range)
    else:
        if axis==1: ax.set_ylim(y_min - 100, y_max + 100)
        elif axis==0: ax.set_xlim(y_min - 100, y_max + 100)

# === Example Usage ===
if __name__ == "__main__":
    if MAKE_MEASUREMENT is True:
        reset_via_pylink()
        time.sleep(1)

        port = "COM16"
        baud = 115200

        with serial.Serial(port, baud, timeout=1) as ser:
            time.sleep(2)  # Wait for serial port to initialize

            voltage_mV = 500
            start_freq = 100
            stop_freq = 1000
            num_points = 10
            log_distribution = False

            config_eis(ser, voltage_mV, start_freq, stop_freq, num_points, log_spacing=log_distribution)

            time.sleep(2)

            start_measurement(ser) # stop automatically if no new data after duration_sec

    frequencies = []
    magnitudes = []
    phases = []
    real_parts = []
    imag_parts = []

    # Read and parse measurement_log.txt
    with open('measurement_log.txt', 'r') as file:
        for line in file:
            parsed = parse_line_calibrated(line, 10000)
            if parsed:
                freq, Zx_re, Zx_im, Zx_mag, Zx_phase_deg = parsed
                #print(f"Freq: {freq} Hz, Zx_re: {Zx_re} Ω, Zx_im: {Zx_im} Ω, Mag: {Zx_mag} Ω, Phase: {Zx_phase_deg:.2f}°")
                frequencies.append(freq)
                real_parts.append(Zx_re)
                imag_parts.append(Zx_im)
                magnitudes.append(Zx_mag)
                phases.append(Zx_phase_deg)

    # Sort by frequency for plotting
    combined = sorted(zip(frequencies, real_parts, imag_parts, magnitudes, phases))
    frequencies, real_parts, imag_parts, magnitudes, phases = zip(*combined)

    # Plot
    plt.figure()
    plt.plot(real_parts, imag_parts, 'o')
    set_min_lim(imag_parts, 1, 1500000)
    set_min_lim(real_parts, 0, 50000)
    plt.xlabel("Real(Z) [Ω]")
    plt.ylabel("Imag(Z) [Ω]")
    plt.title("Nyquist Plot")
    plt.grid(True)


    plt.figure(figsize=(12, 5))

    plt.subplot(1, 2, 1)
    plt.plot(frequencies, magnitudes, marker='o')
    set_min_lim(magnitudes, 1, 100000)
    plt.xscale('log')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Magnitude')
    plt.title('Impedance Magnitude')
    plt.grid(True)

    plt.subplot(1, 2, 2)
    plt.plot(frequencies, phases, marker='o', color='orange')
    set_min_lim(phases, 1, 400)
    plt.xscale('log')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Phase (degrees)')
    plt.title('Impedance Phase')
    plt.grid(True)

    plt.tight_layout()
    plt.show()