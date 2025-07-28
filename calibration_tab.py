from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton, QFileDialog, QTabWidget, QMessageBox, QComboBox
)
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import QUrl
import os
import numpy as np
import plotly.graph_objs as go
import plotly.io as pio
import tempfile

class CalibrationTab(QWidget):
    def __init__(self):
        super().__init__()

        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        #self.label = QLabel("Calibration Curve")
        #self.layout.addWidget(self.label)

        # Frequency input section
        self.freq_layout = QHBoxLayout()
        self.freq_layout.addWidget(QLabel("Start Frequency (Hz):"))
        self.start_freq_input = QLineEdit()
        self.start_freq_input.setText("10000")
        self.freq_layout.addWidget(self.start_freq_input)

        self.freq_layout.addWidget(QLabel("Stop Frequency (Hz):"))
        self.stop_freq_input = QLineEdit()
        self.stop_freq_input.setText("100")
        self.freq_layout.addWidget(self.stop_freq_input)
        self.layout.addLayout(self.freq_layout)

        fit_layout = QHBoxLayout()
        fit_layout.addWidget(QLabel("Fit Type:"))
        self.fit_type_combo = QComboBox()
        self.fit_type_combo.addItems(["Linear", "Quadratic", "Logarithmic"])
        fit_layout.addWidget(self.fit_type_combo)

        self.layout.addLayout(fit_layout)

        self.load_button = QPushButton("Load Calibration Folder")
        self.load_button.clicked.connect(self.load_calibration_data)
        self.layout.addWidget(self.load_button)

        self.web_view = QWebEngineView()
        self.layout.addWidget(self.web_view)

    def extract_concentration(self, filename: str) -> float:
        # Assumes file is named like "sample_350uM.txt"
        import re
        match = re.search(r"(\d+)\s*u?M", filename)
        return float(match.group(1)) if match else 0

    def extract_impedance(self, file_path, start_freq, stop_freq):
        freqs = []
        mags = []

        with open(file_path, 'r') as f:
            for line in f:
                if not line.strip():
                    continue  # Skip empty lines
                try:
                    parts = line.strip().split(',')
                    freq = float(parts[0])
                    mag = float(parts[3])
                    if stop_freq <= freq <= start_freq:
                        freqs.append(freq)
                        mags.append(mag)
                        #print(mag)
                except (IndexError, ValueError):
                    continue  # Skip malformed lines

        if not mags:
            return 0  # Or np.nan to indicate missing data

        return float(np.mean(mags))

    def load_calibration_data(self):
        folder = QFileDialog.getExistingDirectory(self, "Select Calibration Folder")
        if not folder:
            return

        try:
            start_freq = int(self.start_freq_input.text())
            stop_freq = int(self.stop_freq_input.text())
        except ValueError:
            print("Invalid frequency input")
            return

        files = sorted([f for f in os.listdir(folder) if f.endswith(".txt")])
        concentrations = []
        impedances = []

        for fname in files:
            full_path = os.path.join(folder, fname)
            Z = self.extract_impedance(full_path, start_freq, stop_freq)
            conc = self.extract_concentration(fname)
            print(conc)
            if Z > 0:
                concentrations.append(conc)
                impedances.append(Z)

        if not concentrations:
            QMessageBox.warning(self, "No Data", "No valid calibration data found.")
            return

        self.plot_calibration_curve(np.array(concentrations), np.array(impedances))

    def plot_calibration_curve(self, x, y):

        # Fit type selection
        fit_type = self.fit_type_combo.currentText().lower()

        if fit_type == "linear":
            p = np.polyfit(x, y, 1)
            y_fit = np.polyval(p, x)

        elif fit_type == "quadratic":
            p = np.polyfit(x, y, 2)
            y_fit = np.polyval(p, x)

        elif fit_type == "logarithmic":
            from scipy.optimize import curve_fit

            def log_func(x, a, b, c):
                return a * np.log(b * x) + c

            popt, _ = curve_fit(log_func, x, y, bounds=(0, np.inf))
            y_fit = log_func(x, *popt)

        else:
            y_fit = y  # fallback (no fit)

        fig = go.Figure()
        fig.add_trace(go.Scatter(x=x, y=y, mode='markers', name='Measured'))
        fig.add_trace(go.Scatter(x=x, y=y_fit, mode='lines', name=f'{self.fit_type_combo.currentText()} Fit'))

        fig.update_layout(
            title="Calibration Curve: Impedance vs Concentration",
            xaxis_title="Creatinine Concentration (µM)",
            yaxis_title="Impedance (Ω)",
            template="plotly_white",
            height=500,
        )

        tmp_dir = tempfile.gettempdir()
        fig_path = os.path.join(tmp_dir, "fig_plot.html")
        with open(fig_path, 'w', encoding='utf-8') as f:
            f.write(pio.to_html(fig, full_html=True))

        self.web_view.load(QUrl.fromLocalFile(fig_path))