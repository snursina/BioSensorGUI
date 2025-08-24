import sys, os
import threading
import serial
import tempfile
import math
import time

# try:
#     import pyi_splash
# except Exception:
#     pyi_splash = None

import serial.tools.list_ports
from PyQt5.QtWidgets import (
    QApplication, QWidget, QMainWindow, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QComboBox, QLineEdit, QTabWidget, QTextEdit, QFileDialog
)
from PyQt5.QtCore import Qt, pyqtSignal, QObject, pyqtSlot, QUrl, QTimer
import plotly.graph_objs as go
from plotly import io as pio
from PyQt5.QtWebEngineWidgets import QWebEngineView  # Requires PyQtWebEngine
from PyQt5.QtGui import QGuiApplication, QPixmap, QIcon

from measure_plot import (
    reset_via_pylink, config_eis, start_measurement, parse_line_calibrated
)
from calibration_tab import CalibrationTab
#from ctgan_tab import CTGANTab
from bpnn_tab import BPNNTab

if hasattr(sys, "_MEIPASS"): application_path = sys._MEIPASS
elif getattr(sys, 'frozen', False): application_path = os.path.dirname(sys.executable)
elif __file__: application_path = os.path.dirname(__file__)
if hasattr(sys, "_MEIPASS"): file_path = os.path.dirname(sys.executable)
else: file_path = sys.path[0]

class LowPassFilter:
    def __init__(self, factor: int = 8):
        self.factor = factor
        self.filtered_value = None  # Holds last filtered value

    def filter(self, new_sample: float) -> float:
        if self.filtered_value is None:
            self.filtered_value = new_sample * self.factor
        else:
            self.filtered_value = (self.filtered_value - (self.filtered_value / self.factor)) + new_sample
        return self.filtered_value / self.factor
    
    def clear(self):
        self.filtered_value = None

class WorkerSignals(QObject):
    data_ready = pyqtSignal(str)
    status_update = pyqtSignal(str)
    measurement_finished = pyqtSignal()

class MeasurementThread(threading.Thread):
    def __init__(self, ser, signals, simulator_mode=False):
        super().__init__()
        self.ser = ser
        self.signals = signals
        self.simulator_mode = simulator_mode
        self._stop_flag = threading.Event()

    def run(self):
        def callback(line):
            self.signals.data_ready.emit(line)

        if self.simulator_mode:
            # Passive reader: forward whatever arrives from the simulator
            try:
                while not self._stop_flag.is_set():
                    raw = self.ser.readline()
                    if not raw:
                        continue
                    line = raw.decode("ascii", errors="ignore").strip()
                    if line:
                        callback(line)
            finally:
                self.signals.measurement_finished.emit()
        else:
            # Real device path
            from measure_plot import start_measurement
            start_measurement(self.ser, duration_sec=None, line_callback=callback, stop_event=self._stop_flag)
            self.signals.measurement_finished.emit()

    def stop(self):
        self._stop_flag.set()


import sys
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QListWidget, QStackedWidget,
    QHBoxLayout, QVBoxLayout, QLabel, QPushButton, QLineEdit, QComboBox, QStatusBar#, QWebEngineView
)
from PyQt5.QtCore import Qt


class MainApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AixScan")
        self.setWindowIcon(QIcon("app_icon.svg"))
        screen_geometry = QGuiApplication.primaryScreen().availableGeometry()

        # Half dimensions
        self.app_width = int(screen_geometry.width() // 1.5)
        self.app_height = int(screen_geometry.height() // 1.5)

        # Center position
        x = screen_geometry.x() + (screen_geometry.width() - self.app_width) // 1.5
        y = screen_geometry.y() + (screen_geometry.height() - self.app_height) // 1.5

        self.setGeometry(int(x), int(y), self.app_width, self.app_height)

        self.ser = None
        self.measurement_thread = None
        self.data_lines = []

        self.live_real = []
        self.live_imag = []
        self.live_plot_initialized = False

        self.lp_filter_real = LowPassFilter(factor=4)
        self.lp_filter_imag = LowPassFilter(factor=4)
        self.lp_filter_mag = LowPassFilter(factor=4)
        self.lp_filter_phase = LowPassFilter(factor=4)

        self.signals = WorkerSignals()
        self.signals.data_ready.connect(self.handle_new_data)
        self.signals.status_update.connect(self.update_status)
        self.signals.measurement_finished.connect(self.on_measurement_finished)

        # simulator detection flag
        self.simulator_mode = False

        # === Central layout ===
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # === Sidebar tabs ===
        self.tab_list = QListWidget()
        self.tab_list.setFocusPolicy(Qt.NoFocus)
        self.tab_list.setFixedWidth(int(self.app_width/6))
        self.tab_list.addItem("Live Measurement")
        self.tab_list.addItem("Magnitude & Phase")
        self.tab_list.addItem("Load Data")
        self.tab_list.addItem("Curve Fit")
        #self.tab_list.addItem("CTGAN")
        self.tab_list.addItem("Random Forest")
        self.tab_list.setCurrentRow(0)

        # === Stacked content ===
        self.stack = QStackedWidget()
        self.stack.addWidget(self.init_tab_live())
        self.stack.addWidget(self.init_tab_mag_phase())
        self.stack.addWidget(self.init_tab_load_data())
        self.calibration_tab = CalibrationTab(self)
        self.stack.addWidget(self.calibration_tab)
        #self.ctgan_tab = CTGANTab(self)
        #self.stack.addWidget(self.ctgan_tab)
        self.bpnn_tab = BPNNTab(self)
        self.stack.addWidget(self.bpnn_tab)

        self.tab_list.currentRowChanged.connect(self.stack.setCurrentIndex)

        # === Assemble ===
        sidebar_layout = QVBoxLayout()
        sidebar_layout.addWidget(self.tab_list)
        sidebar_layout.addStretch()

        self.last_prediction = QPushButton()
        font = self.last_prediction.font()
        font.setPointSize(11)
        self.last_prediction.setFont(font)
        self.last_prediction.setText(f"Creatinine Concentration:\n---")
        sidebar_layout.addWidget(self.last_prediction)

        logo_label = QLabel()
        logo_pixmap = QPixmap("kidney_final.svg")
        logo_width = self.tab_list.width()*0.99
        logo_pixmap = logo_pixmap.scaled(int(logo_width), int(logo_width*0.9), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        logo_label.setPixmap(logo_pixmap)
        logo_label.setAlignment(Qt.AlignCenter)
        sidebar_layout.addWidget(logo_label)

        sidebar_container = QWidget()
        sidebar_container.setLayout(sidebar_layout)

        main_layout.addWidget(sidebar_container, 0)
        main_layout.addWidget(self.stack, 1)

        # === Status Bar ===
        self.status_bar_label = QLabel("Select COM port to start.")
        self.status_bar = QStatusBar()
        self.status_bar.addWidget(self.status_bar_label)
        self.setStatusBar(self.status_bar)

        # === Styling ===
        self.setStyleSheet("""
            QListWidget {
                background-color: #f0f0f0;
                border: none;
            }
            QListWidget::item {
                padding: 15px;
                margin: 5px;
                border-radius: 5px;
                font-size: 11pt;
            }
            QListWidget::item:selected {
                background-color: #0051a5;
                color: white;
            }
            QLabel {
                font-size: 11pt;
            }
            QPushButton {
                background-color: #0051a5;
                color: white;
                border-radius: 5px;
                padding: 6px 12px;
            }
            QPushButton:hover {
                background-color: #1a63ad;
            }
            QPushButton:disabled {
                background-color: #ccc;
                color: #666;
            }
            QLineEdit, QComboBox {
                padding: 4px;
                border: 1px solid #ccc;
                border-radius: 4px;
            }
            QStatusBar {
                background-color: #0051a5;
                color: white;
            }
            QStatusBar QLabel{
                color: white;
            }
            QListWidget {
                background-color: #f0f0f0;
                border: none;
                margin-top: 20px;
            }
            QListWidget::item {
                padding: 15px;
                margin: 5px;
                border-radius: 5px;
                font-size: 11pt;
            }
            QListWidget::item:selected {
                background-color: #0051a5;
                color: white;
            }
            QListWidget::item:selected:focus {
                outline: none;
            }
        """)

    def init_tab_live(self):
        tab1 = QWidget()
        layout = QVBoxLayout()

        # COM port selector
        hcom = QHBoxLayout()
        hcom.addWidget(QLabel("COM Port:"))
        self.combo_com = QComboBox()
        hcom.addWidget(self.combo_com)
        btn_refresh = QPushButton("Refresh")
        btn_refresh.clicked.connect(self.refresh_com_ports)
        hcom.addWidget(btn_refresh)
        layout.addLayout(hcom)

        # Settings inputs
        hsettings = QHBoxLayout()
        self.input_voltage = QLineEdit("10")  # in mV
        self.input_start_freq = QLineEdit("30000")
        self.input_end_freq = QLineEdit("10000")
        self.input_num_points = QLineEdit("10")
        self.spacing_combo = QComboBox()
        self.spacing_combo.addItems(["Linear", "Logarithmic"])

        hsettings.addWidget(QLabel("Voltage (mV):"))
        hsettings.addWidget(self.input_voltage)
        hsettings.addWidget(QLabel("Start Freq (Hz):"))
        hsettings.addWidget(self.input_start_freq)
        hsettings.addWidget(QLabel("End Freq (Hz):"))
        hsettings.addWidget(self.input_end_freq)
        hsettings.addWidget(QLabel("Num Points:"))
        hsettings.addWidget(self.input_num_points)
        hsettings.addWidget(QLabel("Spacing:"))
        hsettings.addWidget(self.spacing_combo)
        layout.addLayout(hsettings)

        # Connect and config buttons
        hbuttons = QHBoxLayout()
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self.connect_serial)
        hbuttons.addWidget(self.btn_connect)

        self.btn_configure = QPushButton("Configure")
        self.btn_configure.clicked.connect(self.configure_device)
        self.btn_configure.setEnabled(False)
        hbuttons.addWidget(self.btn_configure)

        self.btn_start = QPushButton("Start Measurement")
        self.btn_start.clicked.connect(self.start_measurement)
        self.btn_start.setEnabled(False)
        hbuttons.addWidget(self.btn_start)

        self.btn_stop = QPushButton("Stop Measurement")
        self.btn_stop.clicked.connect(self.stop_measurement)
        self.btn_stop.setEnabled(False)
        hbuttons.addWidget(self.btn_stop)

        layout.addLayout(hbuttons)

        # Live plot area - embed QWebEngineView to show plotly live updates
        self.live_plot_view = QWebEngineView()
        layout.addWidget(self.live_plot_view)

        hsave = QHBoxLayout()
        #hsave.addWidget(QLabel("Prediction:"))
        #self.prediction_box = QPushButton("Prediction: ---")
        #self.prediction_box.setEnabled(False)
        #hsave.addWidget(self.prediction_box)
        hsave.addWidget(QLabel("Save as:"))
        self.save_file_name = QLineEdit("sample_uM")
        self.btn_save = QPushButton("Save Measurement")
        self.btn_save.clicked.connect(self.save_measurement)
        self.btn_save.setEnabled(False)
        self.btn_clear = QPushButton("Clear Plot")
        self.btn_clear.clicked.connect(self.clear_plot)
        hsave.addWidget(self.save_file_name)
        hsave.addWidget(self.btn_save)
        hsave.addWidget(self.btn_clear)
        layout.addLayout(hsave)

        tab1.setLayout(layout)

        self.init_live_plot()
        self.refresh_com_ports()

        return tab1

    def init_tab_mag_phase(self):
        tab2 = QWidget()
        layout = QVBoxLayout()

        self.post_plot_view_mag = QWebEngineView()
        self.post_plot_view_phase = QWebEngineView()
        layout.addWidget(self.post_plot_view_mag)
        layout.addWidget(self.post_plot_view_phase)

        tab2.setLayout(layout)
        return tab2

    def init_tab_load_data(self):
        tab3 = QWidget()
        layout = QVBoxLayout()

        # Buttons row
        hbuttons = QHBoxLayout()
        self.btn_load_file = QPushButton("Load Measurement Log")
        self.btn_load_file.clicked.connect(self.load_data_file)
        hbuttons.addWidget(self.btn_load_file)

        self.btn_clear_nyquist = QPushButton("Clear Plot")
        self.btn_clear_nyquist.clicked.connect(self.clear_nyquist_plot)
        hbuttons.addWidget(self.btn_clear_nyquist)

        layout.addLayout(hbuttons)

        # Nyquist Plot View
        self.nyquist_plot_view = QWebEngineView()
        layout.addWidget(self.nyquist_plot_view)

        # Initialize empty Nyquist plot
        html = """
        <!DOCTYPE html>
        <html>
        <head>
            <meta charset="utf-8" />
            <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
            <style>html, body { height:100%; margin:0; }</style>
        </head>
        <body>
            <div id="plot" style="width:100%; height:100%;"></div>
            <script>
                var layout = {
                    title: { text: "Nyquist Plot (Real vs Imag)", font: { size: 18 } },
                    xaxis: { title: { text: "Real(Z) [Ω]", font: { size: 14 } }, tickfont: { size: 12 } },
                    yaxis: { title: { text: "Imag(Z) [Ω]", font: { size: 14 } }, tickfont: { size: 12 }, scaleanchor: "x", scaleratio: 1 },
                    template: "plotly_white",
                    legend: { font: { size: 12 } },
                    width: 1600,
                    height: 900
                };
                Plotly.newPlot("plot", [], layout);

                function clearPlot() {
                    Plotly.deleteTraces("plot", Array.from({length: document.getElementById("plot").data.length}, (_, i) => i));
                }
            </script>
        </body>
        </html>
        """
        self.nyquist_plot_view.setHtml(html)

        tab3.setLayout(layout)
        return tab3

    def refresh_com_ports(self):
        self.combo_com.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.combo_com.addItem(port.device)

    # ---- Simulator helpers ----
    def _probe_simulator(self, timeout=0.6) -> bool:
        """Listen briefly for SIM:HELLO banner."""
        try:
            if not self.ser:
                return False
            self.ser.reset_input_buffer()
            deadline = time.time() + timeout
            while time.time() < deadline:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="ignore").strip()
                if line.startswith("SIM:HELLO"):
                    return True
        except Exception:
            pass
        return False

    def _parse_live_line(self, line, calib_resistor=1000):
        """Try real parser first; fallback to CSV 'real,imag,freq' (simulator)."""
        try:
            parsed = parse_line_calibrated(line, calib_resistor)
            if parsed:
                return parsed
        except Exception:
            pass
        try:
            parts = [p.strip() for p in line.strip().split(',')]
            if len(parts) >= 3:
                re_val = float(parts[0]); im_val = float(parts[1]); freq = float(parts[2])
                mag = math.hypot(re_val, im_val)
                phase = math.degrees(math.atan2(im_val, re_val))
                return (freq, re_val, im_val, mag, phase)
        except Exception:
            return None
        return None

    def connect_serial(self):
        selected_port = self.combo_com.currentText()
        if not selected_port:
            self.update_status("No COM port selected.")
            return

        try:
            # Open first so we can detect a simulator banner
            self.ser = serial.Serial(selected_port, 115200, timeout=0.1)

            # Quick probe for simulator
            self.simulator_mode = self._probe_simulator(timeout=0.6)

            if self.simulator_mode:
                self.update_status(f"Simulator detected on {selected_port}. Configure is bypassed.")
                self.btn_configure.setEnabled(False)
                self.btn_start.setEnabled(True)
            else:
                # Real device: reset + normal configure flow
                reset_via_pylink()
                self.update_status(f"Connected to {selected_port}")
                self.btn_configure.setEnabled(True)
        except Exception as e:
            self.update_status(f"Failed to connect: {e}")

    def configure_device(self):
        if not self.ser or not self.ser.is_open:
            self.update_status("Serial port not connected.")
            return
        if self.simulator_mode:
            self.update_status("Simulator mode: configuration bypassed.")
            self.btn_start.setEnabled(True)
            return
        try:
            voltage = int(self.input_voltage.text())
            start_freq = int(self.input_start_freq.text())
            end_freq = int(self.input_end_freq.text())
            num_points = int(self.input_num_points.text())
            log_spacing = (self.spacing_combo.currentText() != "Linear")

            config_eis(self.ser, voltage, start_freq, end_freq, num_points, log_spacing)
            self.update_status("Configuration done.")
            self.btn_start.setEnabled(True)
        except Exception as e:
            self.update_status(f"Configuration error: {e}")

    def start_measurement(self):
        if not self.ser or not self.ser.is_open:
            self.update_status("Serial port not connected.")
            return

        self.data_lines.clear()
        self.live_real.clear()
        self.live_imag.clear()
        self.live_plot_initialized = False
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.btn_configure.setEnabled(False)
        self.btn_connect.setEnabled(False)
        self.btn_save.setEnabled(False)

        self.clear_filters()

        self.live_plot_view.page().runJavaScript("startNewRun();")

        # Simulator handshake: ACK + START
        if self.simulator_mode:
            try:
                self.ser.write(b"SIM:ACK\n")
                self.ser.write(b"SIM:START\n")
            except Exception:
                pass

        self.measurement_thread = MeasurementThread(self.ser, self.signals, simulator_mode=self.simulator_mode)
        self.measurement_thread.start()

    def stop_measurement(self):
        # tell simulator to stop streaming
        if self.simulator_mode and self.ser and self.ser.is_open:
            try:
                self.ser.write(b"SIM:STOP\n")
                self.btn_start.setEnabled(True)
            except Exception:
                pass

        if self.measurement_thread:
            self.measurement_thread.stop()
            self.measurement_thread.join()
            self.measurement_thread = None

        self.btn_stop.setEnabled(False)
        self.btn_start.setEnabled(True)
        self.btn_configure.setEnabled(True and not self.simulator_mode)
        self.btn_connect.setEnabled(True)
        self.btn_save.setEnabled(True)

    def save_measurement(self):
        with open(os.path.join(file_path, "saved_data/" + self.save_file_name.text().strip() + ".txt"), "w") as f:
            for freq, real, imag in zip(self.freq_list, self.live_real, self.live_imag):
                f.write(f"{real},{imag}, {freq}\n")

    def clear_plot(self):
        self.live_plot_view.page().runJavaScript("clearPlot();")

    def handle_new_data(self, line):
        self.data_lines.append(line)
        if not self.live_plot_initialized:
            pass
        self.update_live_plot()

    def init_live_plot(self):
        # Plotly with run-based coloring + responsive + queue during init
        html = """
        <!DOCTYPE html>
        <html>
        <head>
            <meta charset="utf-8" />
            <title>Live Nyquist</title>
            <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
            <style>
                html, body { height: 100%; margin: 0; }
                #plot { width: 100%; height: 100%; }
            </style>
        </head>
        <body>
            <div id="plot"></div>
            <script>
                var runIndex = -1;
                var plotReady = false;
                var pendingPoints = [];

                var palette = (Plotly.d3 && Plotly.d3.schemeCategory10) ?
                              Plotly.d3.schemeCategory10 :
                              ['#1f77b4','#ff7f0e','#2ca02c','#d62728','#9467bd',
                               '#8c564b','#e377c2','#7f7f7f','#bcbd22','#17becf'];

                var layout = {
                    title: { text: 'Live Nyquist Plot', font: { size: 18 } },
                    xaxis: { title: { text: 'Real(Z) [Ω]', font: { size: 14 } }, tickfont: { size: 12 }, autorange: true },
                    yaxis: { title: { text: 'Imag(Z) [Ω]', font: { size: 14 } }, tickfont: { size: 12 },
                             autorange: true, scaleanchor: "x", scaleratio: 1 },
                    legend: { font: { size: 12 } },
                    template: 'plotly_white',
                    uirevision: 'live-nyquist',
                    height: 700
                };

                Plotly.newPlot('plot', [], layout).then(function () {
                    plotReady = true;
                    var plt = document.getElementById('plot');
                    if ((!plt.data || !plt.data.length) && pendingPoints.length) {
                        startNewRun();
                    }
                    for (var i = 0; i < pendingPoints.length; i++) {
                        var p = pendingPoints[i];
                        Plotly.extendTraces('plot', { x: [[p.x]], y: [[p.y]] }, [runIndex]);
                    }
                    pendingPoints.length = 0;
                });

                function startNewRun() {
                    if (!plotReady) { return false; }
                    runIndex += 1;
                    var color = palette[runIndex % palette.length];
                    var newTrace = {
                        x: [],
                        y: [],
                        mode: 'lines+markers',
                        name: 'Run ' + (runIndex + 1),
                        marker: { color: color, size: 8 }
                    };
                    Plotly.addTraces('plot', [newTrace]);
                    return true;
                }

                function updateRun(newX, newY) {
                    newX = +newX; newY = +newY;
                    if (!plotReady) {
                        pendingPoints.push({ x: newX, y: newY });
                        return;
                    }
                    var plt = document.getElementById('plot');
                    if (runIndex < 0 || !plt.data || !plt.data.length) {
                        startNewRun();
                    }
                    Plotly.extendTraces('plot', { x: [[newX]], y: [[newY]] }, [runIndex]);

                    var MAX_POINTS = 2000;
                    var d = plt.data[runIndex];
                    if (d && d.x && d.x.length > MAX_POINTS) {
                        Plotly.restyle('plot', {
                            x: [d.x.slice(-MAX_POINTS)],
                            y: [d.y.slice(-MAX_POINTS)]
                        }, [runIndex]);
                    }
                }

                function clearPlot() {
                    var plt = document.getElementById('plot');
                    if (plt.data && plt.data.length) {
                        Plotly.deleteTraces('plot', Array.from({length: plt.data.length}, (_, i) => i));
                    }
                    runIndex = -1;
                }

                window.addEventListener('resize', () => {
                    Plotly.relayout('plot', { width: window.innerWidth, height: window.innerHeight });
                });
            </script>
        </body>
        </html>
        """
        self.live_plot_view.setHtml(html)
        self.live_plot_initialized = True

    def update_live_plot(self):
        calib_resistor = 1000
        line = self.data_lines[-1]
        parsed = self._parse_live_line(line, calib_resistor=calib_resistor)
        if not parsed:
            return

        freq, Zx_re, Zx_im, _, _ = parsed

        Zx_re = self.lp_filter_real.filter(Zx_re)
        Zx_im = self.lp_filter_imag.filter(Zx_im)

        if Zx_im > 0:
            print(f"Real: {Zx_re:.3f}, Capacity: {(1/(freq*2*math.pi*(Zx_im)))}")
        else:
            print(f"Real: {Zx_re:.3f}, Inductor: {(-Zx_im/(freq*2*math.pi))}")
        
        self.live_real.append(Zx_re)
        self.live_imag.append(Zx_im)

        js_code = f"updateRun({Zx_re:.3f}, {Zx_im:.3f});"
        self.live_plot_view.page().runJavaScript(js_code)

    def on_measurement_finished(self):
        self.freq_list = []
        mag_list = []
        phase_list = []

        calib_resistor = 1000
        for line in self.data_lines:
            parsed = self._parse_live_line(line, calib_resistor)
            if parsed:
                freq, _, _, mag, phase = parsed
                mag = self.lp_filter_mag.filter(mag)
                phase = self.lp_filter_phase.filter(phase)
                self.freq_list.append(freq)
                mag_list.append(mag)
                phase_list.append(phase)

        if not self.freq_list:
            self.update_status("No data to plot in Magnitude & Phase.")
            return

        mag_min = min(mag_list)
        mag_max = max(mag_list)
        y_range_mag = [max(0, mag_min - 10000), mag_max + mag_max*0.5]

        fig_mag = go.Figure()
        fig_mag.add_trace(go.Scatter(x=self.freq_list, y=mag_list, mode='lines+markers', name='Magnitude (Ω)'))
        fig_mag.update_layout(
            title=dict(text="Magnitude vs Frequency", font=dict(size=18)),
            xaxis=dict(title=dict(text="Frequency (Hz)", font=dict(size=14)), tickfont=dict(size=12)),
            yaxis=dict(title=dict(text="Magnitude (Ω)", font=dict(size=14)), tickfont=dict(size=12)),
            template="plotly_white",
            height=int(self.app_height/2),
        )
        fig_mag.update_yaxes(range=y_range_mag)

        fig_phase = go.Figure()
        fig_phase.add_trace(go.Scatter(x=self.freq_list, y=phase_list, mode='lines+markers', name='Phase (°)', line=dict(color='orange')))
        fig_phase.update_layout(
            title=dict(text="Phase vs Frequency", font=dict(size=18)),
            xaxis=dict(title=dict(text="Frequency (Hz)", font=dict(size=14)), tickfont=dict(size=12)),
            yaxis=dict(title=dict(text="Phase (°)", font=dict(size=14)), tickfont=dict(size=12)),
            template="plotly_white",
            height=int(self.app_height/2),
        )
        fig_phase.update_yaxes(range=[-190, 190])

        tmp_dir = tempfile.gettempdir()
        mag_path = os.path.join(tmp_dir, "mag_plot.html")
        phase_path = os.path.join(tmp_dir, "phase_plot.html")

        with open(mag_path, 'w', encoding='utf-8') as f:
            f.write(pio.to_html(fig_mag, full_html=True))

        with open(phase_path, 'w', encoding='utf-8') as f:
            f.write(pio.to_html(fig_phase, full_html=True))

        self.post_plot_view_mag.load(QUrl.fromLocalFile(mag_path))
        self.post_plot_view_phase.load(QUrl.fromLocalFile(phase_path))

        self.disconnect_serial()
        with open(os.path.join(file_path, "measurement_log.txt"), "w") as f:
            for real, imag, freq in zip(self.live_real, self.live_imag, self.freq_list):
                f.write(f"{real},{imag},{freq}\n")
        self.btn_save.setEnabled(True)
        self.connect_serial()

        try:
            if self.bpnn_tab.model is None: self.bpnn_tab.btnLoadModel.click()
            pred = self.bpnn_tab.predict_single(os.path.join(file_path, "measurement_log.txt"))
            self.last_prediction.setText(f"Creatinine Concentration:\n{pred:.2f} µM")
            self.update_status("Measurement finished and prediction updated.")
        except: self.update_status("Measurement finished but prediction cannot be updated.")

    def update_status(self, msg):
        self.status_bar.showMessage(msg)

    def disconnect_serial(self):
        if self.ser and self.ser.is_open:
            try:
                # send stop to simulator on disconnect just in case
                if self.simulator_mode:
                    try:
                        self.ser.write(b"SIM:STOP\n")
                    except Exception:
                        pass
                self.ser.close()
                self.update_status("Serial port disconnected.")
            except Exception as e:
                self.update_status(f"Error closing serial port: {e}")
        self.ser = None
        self.btn_connect.setEnabled(True)
        self.btn_configure.setEnabled(False)
        if not self.simulator_mode: self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(False)
        
    def clear_nyquist_plot(self):
        self.nyquist_plot_view.page().runJavaScript("clearPlot();")
        self.update_status("Nyquist plot cleared.")

    def load_data_file(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "Open Measurement Log", "", "Text Files (*.txt *.log);;All Files (*)")
        if not file_path:
            return

        real_vals, imag_vals = [], []
        with open(file_path, "r") as f:
            for line in f:
                parts = line.strip().split(',')
                if len(parts) < 3:
                    continue
                try:
                    real = float(parts[0])
                    imag = float(parts[1])
                    real_vals.append(real)
                    imag_vals.append(imag)
                except ValueError:
                    continue

        # Build JS trace
        import json, os
        trace_data = {
            "x": real_vals,
            "y": imag_vals,
            "mode": "markers+lines",
            "name": os.path.basename(file_path)  # label = filename
        }
        js_code = f"Plotly.addTraces('plot', [{json.dumps(trace_data)}]);"
        self.nyquist_plot_view.page().runJavaScript(js_code)

        self.update_status(f"Added data from {os.path.basename(file_path)}")


    def clear_filters(self):
        self.lp_filter_real.clear()
        self.lp_filter_imag.clear()
        self.lp_filter_mag.clear()
        self.lp_filter_phase.clear()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.live_plot_view.page().runJavaScript("""
            Plotly.relayout('plot', {
                width: window.innerWidth,
                height: window.innerHeight
            });
        """)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainApp()
    window.show()

    def _close_splash():
        try:
            import pyi_splash
            pyi_splash.close()
        except Exception:
            pass

    # Defer closing to the next event tick so Qt can repaint
    QTimer.singleShot(0, _close_splash)

    sys.exit(app.exec_())
