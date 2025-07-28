import sys, os
import threading
import serial
import tempfile
import serial.tools.list_ports
from PyQt5.QtWidgets import (
    QApplication, QWidget, QMainWindow, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QComboBox, QLineEdit, QTabWidget, QTextEdit, QFileDialog
)
from PyQt5.QtCore import Qt, pyqtSignal, QObject, pyqtSlot, QUrl
import plotly.graph_objs as go
from plotly import io as pio
from PyQt5.QtWebEngineWidgets import QWebEngineView  # Requires PyQtWebEngine

from measure_plot import (
    reset_via_pylink, config_eis, start_measurement, parse_line_calibrated
)
from calibration_tab import CalibrationTab

class LowPassFilter:
    def __init__(self, factor: int = 8):
        self.factor = factor
        self.filtered_value = None  # Holds last filtered value

    def filter(self, new_sample: float) -> float:
        if self.filtered_value is None:
            # Initialize with the first sample
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
    def __init__(self, ser, signals):
        super().__init__()
        self.ser = ser
        self.signals = signals
        self._stop_flag = threading.Event()

    def run(self):
        def callback(line):
            self.signals.data_ready.emit(line)

        from measure_plot import start_measurement
        start_measurement(self.ser, duration_sec=None, line_callback=callback, stop_event=self._stop_flag)

        self.signals.measurement_finished.emit()

    def stop(self):
        self._stop_flag.set()

class MainApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("EIS Measurement GUI")
        self.setGeometry(100, 100, 1280, 1000)

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

        self.init_ui()

    def init_ui(self):
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        self.init_tab1()
        self.init_tab2()
        self.init_tab3()

        self.calibration_tab = CalibrationTab()
        self.tabs.addTab(self.calibration_tab, "Calibration")

        self.status_bar = QLabel("Select COM port to start.")
        self.statusBar().addWidget(self.status_bar)

    def init_tab1(self):
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
        self.input_voltage = QLineEdit("20")  # in mV
        self.input_start_freq = QLineEdit("10000")
        self.input_end_freq = QLineEdit("100")
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

        tab1.setLayout(layout)
        self.tabs.addTab(tab1, "Live Measurement")

        self.refresh_com_ports()

    def init_tab2(self):
        tab2 = QWidget()
        layout = QVBoxLayout()

        # Post-measurement plot
        self.post_plot_view_mag = QWebEngineView()
        self.post_plot_view_phase = QWebEngineView()
        layout.addWidget(self.post_plot_view_mag)
        layout.addWidget(self.post_plot_view_phase)

        tab2.setLayout(layout)
        self.tabs.addTab(tab2, "Magnitude & Phase")

    def init_tab3(self):
        tab3 = QWidget()
        layout = QVBoxLayout()

        self.btn_load_file = QPushButton("Load Measurement Log")
        self.btn_load_file.clicked.connect(self.load_data_file)
        layout.addWidget(self.btn_load_file)

        self.nyquist_plot_view = QWebEngineView()
        self.load_plot_view_mag = QWebEngineView()
        self.load_plot_view_phase = QWebEngineView()
        
        layout.addWidget(self.nyquist_plot_view)
        layout.addWidget(self.load_plot_view_mag)
        layout.addWidget(self.load_plot_view_phase)

        tab3.setLayout(layout)
        self.tabs.addTab(tab3, "Load Data")

    def refresh_com_ports(self):
        self.combo_com.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.combo_com.addItem(port.device)

    def connect_serial(self):
        selected_port = self.combo_com.currentText()
        if not selected_port:
            self.update_status("No COM port selected.")
            return

        try:
            # Reset device before connecting
            reset_via_pylink()

            self.ser = serial.Serial(selected_port, 115200, timeout=0.1)
            self.update_status(f"Connected to {selected_port}")
            self.btn_configure.setEnabled(True)
        except Exception as e:
            self.update_status(f"Failed to connect: {e}")

    def configure_device(self):
        if not self.ser or not self.ser.is_open:
            self.update_status("Serial port not connected.")
            return
        try:
            voltage = int(self.input_voltage.text())
            start_freq = int(self.input_start_freq.text())
            end_freq = int(self.input_end_freq.text())
            num_points = int(self.input_num_points.text())
            if self.spacing_combo.currentText() == "Linear": log_spacing = False
            else: log_spacing = True

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

        self.clear_filters()

        self.measurement_thread = MeasurementThread(self.ser, self.signals)
        self.measurement_thread.start()

    def stop_measurement(self):
        if self.measurement_thread:
            self.measurement_thread.stop()
            self.measurement_thread.join()
            self.measurement_thread = None

        self.btn_stop.setEnabled(False)
        self.btn_start.setEnabled(True)
        self.btn_configure.setEnabled(True)
        self.btn_connect.setEnabled(True)

    def handle_new_data(self, line):
        self.data_lines.append(line)

        if not self.live_plot_initialized:
            self.init_live_plot()

        self.update_live_plot()

    def init_live_plot(self):
        # Load the initial empty Plotly HTML with JS update function
        html = """
        <!DOCTYPE html>
        <html>
        <head>
            <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
        </head>
        <body>
            <div id="plot" style="width:100%; height:100%;"></div>
            <script>
                var trace = {
                    x: [],
                    y: [],
                    mode: 'lines+markers',
                    name: 'Impedance'
                };
                var layout = {
                    title: 'Live Nyquist Plot',
                    xaxis: { title: 'Real(Z) [Ω]' },
                    yaxis: { title: 'Imag(Z) [Ω]', scaleanchor: "x", scaleratio: 1 },
                    template: 'plotly_white',
                    height: 700
                };
                Plotly.newPlot('plot', [trace], layout);

                function updatePlot(newX, newY) {
                    Plotly.extendTraces('plot', {
                        x: [[newX]],
                        y: [[newY]]
                    }, [0]);

                    var maxPoints = 100;
                    var data = document.getElementById('plot').data[0];
                    if(data.x.length > maxPoints) {
                        Plotly.relayout('plot', {
                            'xaxis.range': [data.x[data.x.length - maxPoints], data.x[data.x.length - 1]],
                            'yaxis.range': [Math.min(...data.y.slice(-maxPoints)), Math.max(...data.y.slice(-maxPoints))]
                        });
                    }
                }
            </script>
        </body>
        </html>
        """
        self.live_plot_view.setHtml(html)
        self.live_plot_initialized = True

    def update_live_plot(self):
        # Called on every new data line.
        # Parse the latest data point and send to JS
        calib_resistor = 10000  # same as before

        # Parse only the last line to avoid lagging behind
        line = self.data_lines[-1]

        parsed = parse_line_calibrated(line, calib_resistor)
        if not parsed:
            return

        freq, Zx_re, Zx_im, _, _ = parsed

        Zx_re = self.lp_filter_real.filter(Zx_re)
        Zx_im = self.lp_filter_imag.filter(Zx_im)

        # Append locally for logging
        self.live_real.append(Zx_re)
        self.live_imag.append(Zx_im)

        # Send just this point to JS plot
        js_code = f"updatePlot({Zx_re:.3f}, {Zx_im:.3f});"
        self.live_plot_view.page().runJavaScript(js_code)


    def on_measurement_finished(self):
        freq_list = []
        mag_list = []
        phase_list = []

        calib_resistor = 10000
        for line in self.data_lines:
            parsed = parse_line_calibrated(line, calib_resistor)
            if parsed:
                freq, _, _, mag, phase = parsed

                mag = self.lp_filter_mag.filter(mag)
                phase = self.lp_filter_phase.filter(phase)

                freq_list.append(freq)
                mag_list.append(mag)
                phase_list.append(phase)

        if not freq_list:
            self.update_status("No data to plot in Magnitude & Phase.")
            return

        mag_min = min(mag_list)
        mag_max = max(mag_list)
        y_range_mag = [max(0, mag_min - 10000), mag_max + mag_max*0.5]

        # Plot Magnitude vs Frequency
        fig_mag = go.Figure()
        fig_mag.add_trace(go.Scatter(x=freq_list, y=mag_list, mode='lines+markers', name='Magnitude (Ω)'))
        fig_mag.update_layout(
            title="Magnitude vs Frequency",
            xaxis_title="Frequency (Hz)",
            yaxis_title="Magnitude (Ω)",
            template="plotly_white",
            height=400,
        )
        fig_mag.update_yaxes(range=y_range_mag)

        # Plot Phase vs Frequency
        fig_phase = go.Figure()
        fig_phase.add_trace(go.Scatter(x=freq_list, y=phase_list, mode='lines+markers', name='Phase (°)', line=dict(color='orange')))
        fig_phase.update_layout(
            title="Phase vs Frequency",
            xaxis_title="Frequency (Hz)",
            yaxis_title="Phase (°)",
            template="plotly_white",
            height=400,
        )
        fig_phase.update_yaxes(range=[-190, 190])

        # Write temp files and load them into QWebEngineView
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
        with open("measurement_log.txt", "w") as f:
            for freq, real, imag, mag, phase in zip(freq_list, self.live_real, self.live_imag, mag_list, phase_list):
                f.write(f"{freq},{real},{imag},{mag},{phase}\n")
        #time.sleep(0.5)
        self.connect_serial()

        self.update_status("Measurement finished and plots updated.")


    def update_status(self, msg):
        self.status_bar.setText(msg)

    def disconnect_serial(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
                self.update_status("Serial port disconnected.")
            except Exception as e:
                self.update_status(f"Error closing serial port: {e}")
        self.ser = None
        # Update buttons
        self.btn_connect.setEnabled(True)
        self.btn_configure.setEnabled(False)
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(False)

    def load_data_file(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "Open Measurement Log", "", "Text Files (*.txt *.log);;All Files (*)")
        if not file_path:
            return

        with open(file_path, "r") as f:
            lines = f.readlines()

        freq_list, real_vals, imag_vals, mag_list, phase_list = [], [], [], [], []

        with open(file_path, "r") as f:
            for line in f:
                parts = line.strip().split(',')
                if len(parts) != 5:
                    continue
                try:
                    freq = float(parts[0])
                    real = float(parts[1])
                    imag = float(parts[2])
                    mag = float(parts[3])
                    phase = float(parts[4])

                    freq_list.append(freq)
                    real_vals.append(real)
                    imag_vals.append(imag)
                    mag_list.append(mag)
                    phase_list.append(phase)
                except ValueError:
                    continue

        # Nyquist Plot
        fig_nyquist = go.Figure()
        fig_nyquist.add_trace(go.Scatter(x=real_vals, y=imag_vals, mode='markers+lines', name='Impedance'))
        fig_nyquist.update_layout(
            title="Nyquist Plot (Real vs Imag)",
            xaxis_title="Real(Z) [Ω]",
            yaxis_title="Imag(Z) [Ω]",
            yaxis=dict(scaleanchor="x", scaleratio=1),
            template="plotly_white",
            height=300,
        )
        #html_nyquist = pio.to_html(fig_nyquist, full_html=False)

        # Mag and Phase Plot
        fig_mag = go.Figure()
        mag_min = min(mag_list)
        mag_max = max(mag_list)
        mag_pad = 10000
        fig_mag.add_trace(go.Scatter(x=freq_list, y=mag_list, mode='lines+markers', name='Magnitude (Ω)', line=dict(color='blue')))
        fig_mag.update_layout(
            title="Magnitude vs Frequency",
            xaxis_title="Frequency (Hz)",
            yaxis_title="Magnitude (Ω)",
            yaxis=dict(range=[mag_min - mag_pad, mag_max + mag_pad]),
            template="plotly_white",
            height=250,
        )

        fig_phase = go.Figure()
        phase_min = min(phase_list)
        phase_max = max(phase_list)
        fig_phase.add_trace(go.Scatter(x=freq_list, y=phase_list, mode='lines+markers', name='Phase (°)', line=dict(color='orange')))
        fig_phase.update_layout(
            title="Phase vs Frequency",
            xaxis_title="Frequency (Hz)",
            yaxis_title="Phase (°)",
            yaxis=dict(range=[phase_min - 5, phase_max + 5]),
            template="plotly_white",
            height=250,
        )

        # Write temp files and load them into QWebEngineView
        tmp_dir = tempfile.gettempdir()
        nyq_path = os.path.join(tmp_dir, "nyq_plot_load.html")
        mag_path = os.path.join(tmp_dir, "mag_plot_load.html")
        phase_path = os.path.join(tmp_dir, "phase_plot_load.html")

        with open(nyq_path, 'w', encoding='utf-8') as f:
            f.write(pio.to_html(fig_nyquist, full_html=True))
        
        with open(mag_path, 'w', encoding='utf-8') as f:
            f.write(pio.to_html(fig_mag, full_html=True))

        with open(phase_path, 'w', encoding='utf-8') as f:
            f.write(pio.to_html(fig_phase, full_html=True))

        self.nyquist_plot_view.load(QUrl.fromLocalFile(nyq_path))
        self.load_plot_view_mag.load(QUrl.fromLocalFile(mag_path))
        self.load_plot_view_phase.load(QUrl.fromLocalFile(phase_path))

        self.update_status("Plots updated.")

    def clear_filters(self):
        self.lp_filter_real.clear()
        self.lp_filter_imag.clear()
        self.lp_filter_mag.clear()
        self.lp_filter_phase.clear()

    

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainApp()
    window.show()
    sys.exit(app.exec_())
