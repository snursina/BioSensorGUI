# --- ctgan_tab.py ---
import os, re, glob, json, pickle, sys
from pathlib import Path

import numpy as np
import pandas as pd

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QFileDialog,
    QLabel, QComboBox, QSpinBox, QTableView, QMessageBox, QLineEdit
)
from PyQt5.QtGui import QStandardItemModel, QStandardItem
from PyQt5.QtCore import Qt

# SDV / CTGAN
from sdv.single_table import CTGANSynthesizer
from sdv.metadata import SingleTableMetadata


# ----------------- app / file path detection -----------------
if hasattr(sys, "_MEIPASS"):
    application_path = sys._MEIPASS
elif getattr(sys, 'frozen', False):
    application_path = os.path.dirname(sys.executable)
elif __file__:
    application_path = os.path.dirname(__file__)

if hasattr(sys, "_MEIPASS"):
    file_path = os.path.dirname(sys.executable)
else:
    file_path = sys.path[0]
# --------------------------------------------------------------


class CTGANTab(QWidget):
    """
    - Load folder of *.txt files; each file holds rows: real,imag,freq (comma-separated, no header)
    - Parse concentration from filename (uM/mM/nM/M).
    - Train CTGAN on rows: real, imag, FREQ_LOG_R3, concentration_uM (row-wise).
    - Save/load the latest trained model so you don't have to retrain.
    - Generate N synthetic spectra for a chosen concentration using that conc's representative frequency grid.
    - Save generated spectra into <loaded_folder>/gen_data/ as real,imag,freq (no header).
    """
    def __init__(self, parent=None):
        super().__init__(parent)

        self.loaded_folder = None
        self.df = pd.DataFrame()
        self.model = None

        # frequency/grid helpers
        self.file_freq = {}              # file_id -> np.array of freq
        self.rep_freq_by_conc = {}       # concentration_uM -> representative freq grid (np.array)
        self.files_by_conc = {}          # concentration_uM -> [file_ids]
        self.seen_bins_by_conc = {}      # concentration_uM -> sorted unique freq_log_r3 bins (np.array)

        # Where to persist the latest model + aux info (next to app)
        self.model_dir = Path(file_path) / ".aixsense_ctgan"
        self.model_dir.mkdir(parents=True, exist_ok=True)
        self.model_path = self.model_dir / "ctgan_latest.sdv"       # SDV native
        self.pkl_path   = self.model_dir / "ctgan_latest.pkl"       # pickle fallback
        self.aux_path   = self.model_dir / "ctgan_latest_aux.json"  # small extras

        # --- UI ---
        root = QVBoxLayout(self)

        row_btns = QHBoxLayout()
        self.btnLoad = QPushButton("Load folder…")
        self.btnLoadModel = QPushButton("Load model")
        self.btnTrain = QPushButton("Train CTGAN")
        self.btnGen = QPushButton("Generate")
        row_btns.addWidget(self.btnLoad)
        row_btns.addWidget(self.btnLoadModel)
        row_btns.addWidget(self.btnTrain)
        row_btns.addWidget(self.btnGen)
        row_btns.addStretch(1)
        root.addLayout(row_btns)

        row_opts = QHBoxLayout()
        row_opts.addWidget(QLabel("Concentration (µM):"))
        self.cboConc = QComboBox(); self.cboConc.setMinimumWidth(140)
        row_opts.addWidget(self.cboConc)

        row_opts.addWidget(QLabel("Files to generate:"))
        self.spnN = QSpinBox(); self.spnN.setRange(1, 2000); self.spnN.setValue(1)
        row_opts.addWidget(self.spnN)

        row_opts.addWidget(QLabel("Filename prefix:"))
        self.edPrefix = QLineEdit("synthetic")
        self.edPrefix.setMaximumWidth(200)
        row_opts.addWidget(self.edPrefix)

        row_opts.addStretch(1)
        root.addLayout(row_opts)

        self.table = QTableView()
        root.addWidget(self.table)

        # signals
        self.btnLoad.clicked.connect(self.load_folder)
        self.btnLoadModel.clicked.connect(self.on_load_model_clicked)
        self.btnTrain.clicked.connect(self.train_model)
        self.btnGen.clicked.connect(self.generate_and_save)

    # ---------- Helpers ----------

    def parse_conc_from_name(self, name):
        """
        Extract concentration from filename like:
          sample_50uM.txt, data-0.5mM.csv, foo200nM.txt, 1M_*.txt
        Returns µM as float or None if not found.
        """
        base = os.path.splitext(os.path.basename(name))[0]
        m = re.search(r'(\d+(?:\.\d+)?)(?:\s*)(nM|uM|µM|mM|M)\b', base, re.IGNORECASE)
        if not m:
            return None
        val = float(m.group(1))
        unit = m.group(2).lower()
        if unit == 'nm':
            return val / 1000.0            # nM -> µM
        if unit in ('um', 'µm'):
            return val                      # µM
        if unit == 'mm':
            return val * 1000.0             # mM -> µM
        if unit == 'm':
            return val * 1_000_000.0        # M -> µM
        return None

    def _show_preview(self, df: pd.DataFrame, rows=30):
        model = QStandardItemModel(self)
        if df is not None and not df.empty:
            head = df.head(rows)
            model.setColumnCount(len(head.columns))
            model.setHorizontalHeaderLabels(list(head.columns))
            for row in head.itertuples(index=False):
                items = [QStandardItem(str(v)) for v in row]
                model.appendRow(items)
        self.table.setModel(model)
        self.table.resizeColumnsToContents()

    def _pretty_uM(self, conc_uM: float):
        s = f"{conc_uM:.6f}".rstrip("0").rstrip(".")
        return f"{s}uM"

    def _try_load_model(self):
        """Try to load last-saved model + aux (rep grids, seen bins)."""
        if self.model is not None:
            return True
        # Native SDV load
        try:
            if self.model_path.exists():
                self.model = CTGANSynthesizer.load(str(self.model_path))
        except Exception:
            self.model = None
        # Pickle fallback
        if self.model is None and self.pkl_path.exists():
            try:
                with open(self.pkl_path, "rb") as f:
                    self.model = pickle.load(f)
            except Exception:
                self.model = None
        # Aux load
        try:
            if self.aux_path.exists():
                aux = json.loads(self.aux_path.read_text())
                if not self.rep_freq_by_conc and "rep_freq_by_conc" in aux:
                    self.rep_freq_by_conc = {float(k): np.array(v, dtype=float)
                                             for k, v in aux["rep_freq_by_conc"].items()}
                if "seen_bins_by_conc" in aux:
                    self.seen_bins_by_conc = {float(k): np.array(v, dtype=float)
                                              for k, v in aux["seen_bins_by_conc"].items()}
        except Exception as e:
            print("Aux load failed:", e)
        return self.model is not None

    # ---------- Actions ----------

    def load_folder(self):
        folder = QFileDialog.getExistingDirectory(self, "Select data folder", "")
        if not folder:
            return

        frames = []
        self.file_freq.clear()
        self.rep_freq_by_conc.clear()
        self.files_by_conc.clear()
        self.seen_bins_by_conc.clear()
        self.loaded_folder = folder

        # read *.txt as real,imag,freq (comma separated, no header)
        for path in glob.glob(os.path.join(folder, "**", "*.txt"), recursive=True):
            conc_uM = self.parse_conc_from_name(path)
            if conc_uM is None:
                continue

            df = pd.read_csv(path, header=None, names=["real", "imag", "freq"])
            # ensure numeric
            for c in ["real", "imag", "freq"]:
                df[c] = pd.to_numeric(df[c], errors="coerce")
            df = df.dropna(subset=["real", "imag", "freq"])
            if df.empty:
                continue

            df["concentration_uM"] = conc_uM
            file_id = os.path.basename(path)
            df["file_id"] = file_id

            freq = df["freq"].to_numpy()
            self.file_freq[file_id] = freq
            if conc_uM not in self.rep_freq_by_conc:
                self.rep_freq_by_conc[conc_uM] = freq
            self.files_by_conc.setdefault(conc_uM, []).append(file_id)
            frames.append(df)

        if not frames:
            QMessageBox.warning(self, "No data",
                                "No valid *.txt files with concentration in filename were found.")
            return

        self.df = pd.concat(frames, ignore_index=True)

        # Discretized frequency for CTGAN (log + round)
        self.df["freq_log_r3"] = np.log10(self.df["freq"].clip(lower=1e-12)).round(3)

        # Record seen bins per concentration (as floats, sorted)
        self.seen_bins_by_conc = {
            float(c): np.sort(self.df.loc[self.df["concentration_uM"] == c,
                                          "freq_log_r3"].unique().astype(float))
            for c in self.df["concentration_uM"].unique()
        }

        # fill concentration combo
        self.cboConc.clear()
        for c in sorted(self.rep_freq_by_conc.keys()):
            self.cboConc.addItem(f"{c:g}", c)

        self._show_preview(self.df)
        QMessageBox.information(self, "Loaded",
            f"Loaded {len(self.files_by_conc)} concentrations, "
            f"{self.df['file_id'].nunique()} files, {len(self.df)} rows.")

    def on_load_model_clicked(self):
        if self._try_load_model():
            QMessageBox.information(self, "Model loaded",
                                    f"Loaded model from:\n{self.model_path if self.model_path.exists() else self.pkl_path}")
        else:
            QMessageBox.warning(self, "Load failed",
                                "No saved model found, or load failed. Please train first.")

    def train_model(self):
        if self.df.empty:
            QMessageBox.information(self, "Load data", "Load a folder first.")
            return

        # Train on: real, imag, freq_log_r3 (categorical), concentration_uM
        train_cols = ["real", "imag", "freq_log_r3", "concentration_uM"]

        md = SingleTableMetadata()
        md.detect_from_dataframe(self.df[train_cols])
        # Critical typing: freq_log_r3 as categorical, others numerical
        try:
            md.update_column(column_name="freq_log_r3", sdtype="categorical")
        except Exception:
            pass
        for col in ["real", "imag", "concentration_uM"]:
            try:
                md.update_column(column_name=col, sdtype="numerical")
            except Exception:
                pass

        n_rows = len(self.df)
        bs = min(256, max(64, n_rows))

        self.model = CTGANSynthesizer(
            md,
            epochs=300,
            batch_size=bs,
            pac=1,        # avoids divisibility asserts
            verbose=True
        )
        self.model.fit(self.df[train_cols])

        # Save the model
        saved_ok = False
        try:
            self.model.save(str(self.model_path))
            saved_ok = True
        except Exception as e:
            print("SDV save() failed:", e)
            try:
                with open(self.pkl_path, "wb") as f:
                    pickle.dump(self.model, f, protocol=pickle.HIGHEST_PROTOCOL)
                saved_ok = True
            except Exception as e2:
                print("Pickle save failed:", e2)

        # Save aux data (rep grids + seen bins)
        try:
            aux = {
                "rep_freq_by_conc": {str(k): list(map(float, v)) for k, v in self.rep_freq_by_conc.items()},
                "train_cols": train_cols,
                "note": "Trained on freq_log_r3 (categorical)",
                "seen_bins_by_conc": {str(k): list(map(float, v)) for k, v in self.seen_bins_by_conc.items()}
            }
            self.aux_path.write_text(json.dumps(aux))
        except Exception as e:
            print("Aux save failed:", e)

        msg = "CTGAN trained."
        if saved_ok:
            msg += f"\nSaved model to: {self.model_path if self.model_path.exists() else self.pkl_path}"
        QMessageBox.information(self, "Training complete", msg)

    def generate_and_save(self):
        # ensure model available
        if self.model is None:
            if not self._try_load_model():
                QMessageBox.information(self, "No model",
                                        "No trained model found. Please train CTGAN first.")
                return
        if (not self.loaded_folder) and (not self.rep_freq_by_conc):
            QMessageBox.information(self, "Load first",
                                    "Load a data folder first (needed for frequency grids).")
            return
        if self.cboConc.count() == 0:
            QMessageBox.warning(self, "No concentrations", "No concentrations available.")
            return

        conc = float(self.cboConc.currentData())
        n_files = self.spnN.value()
        prefix = (self.edPrefix.text() or "synthetic").strip()

        grid = self.rep_freq_by_conc.get(conc)
        if grid is None or len(grid) == 0:
            QMessageBox.warning(self, "No frequency grid",
                                "No representative frequency grid found for this concentration.")
            return

        # Map the exact grid to log10-rounded bins
        grid_log_r3 = np.log10(np.clip(grid.astype(float), 1e-12, None)).round(3)

        # Snap bins to the nearest seen bin from training for THIS concentration
        seen = self.seen_bins_by_conc.get(conc)
        if seen is None or len(seen) == 0:
            QMessageBox.warning(self, "No seen bins",
                                "No frequency bins recorded for this concentration in training.")
            return

        def snap_to_seen(x):
            idx = np.searchsorted(seen, x)
            if idx == 0:
                return float(seen[0])
            if idx == len(seen):
                return float(seen[-1])
            left, right = seen[idx - 1], seen[idx]
            return float(left if abs(x - left) <= abs(right - x) else right)

        snapped_bins = np.array([snap_to_seen(x) for x in grid_log_r3], dtype=float)

        out_dir = os.path.join(self.loaded_folder or os.getcwd(), "gen_data")
        os.makedirs(out_dir, exist_ok=True)

        # training subset for fallback (same conc)
        subtrain = self.df[self.df["concentration_uM"] == conc][["real", "imag", "freq", "freq_log_r3"]].reset_index(drop=True)

        written = []
        for i in range(1, n_files + 1):
            cond = pd.DataFrame({
                "concentration_uM": np.full_like(snapped_bins, conc, dtype=float),
                # use object dtype to keep categorical tokens exact
                "freq_log_r3": snapped_bins.astype(object)
            })

            parts = []
            chunk_size = 32  # smaller chunks = fewer stalls
            for start in range(0, len(cond), chunk_size):
                sub = cond.iloc[start:start + chunk_size].reset_index(drop=True)
                try:
                    got = self.model.sample_remaining_columns(
                        known_columns=sub,
                        max_tries_per_batch=120   # fail fast; we'll fill holes
                    )
                except Exception:
                    # force fallback path
                    got = pd.DataFrame(columns=["real", "imag", "freq_log_r3", "concentration_uM"])

                # align on (freq_log_r3, conc)
                sub["_key"] = sub["freq_log_r3"].astype(str) + "|" + sub["concentration_uM"].round(9).astype(str)
                # If model didn't return those columns, synthesize keys from sub to keep merge working
                if "freq_log_r3" not in got.columns:
                    got["freq_log_r3"] = sub["freq_log_r3"]
                if "concentration_uM" not in got.columns:
                    got["concentration_uM"] = sub["concentration_uM"]
                got["_key"] = got["freq_log_r3"].astype(str) + "|" + got["concentration_uM"].round(9).astype(str)

                merged = sub.merge(got[["_key", "real", "imag"]], on="_key", how="left")

                # fallback for missing rows: nearest real by raw freq (same conc) + tiny jitter
                if merged["real"].isna().any():
                    miss_idx = np.where(merged["real"].isna())[0]
                    for mi in miss_idx:
                        idx_global = start + mi
                        f_target = float(grid[idx_global])
                        j = (subtrain["freq"] - f_target).abs().idxmin()
                        r0, i0 = float(subtrain.loc[j, "real"]), float(subtrain.loc[j, "imag"])
                        eps = 1e-6
                        merged.loc[mi, "real"] = r0 + np.random.normal(0, max(eps, 0.01 * abs(r0)))
                        merged.loc[mi, "imag"] = i0 + np.random.normal(0, max(eps, 0.01 * abs(i0)))

                parts.append(merged[["freq_log_r3", "real", "imag", "concentration_uM"]])

            synth_all = pd.concat(parts, ignore_index=True)

            # Save as real,imag,freq with your original grid (not snapped)
            out_df = pd.DataFrame({
                "real": synth_all["real"].astype(float).to_numpy(),
                "imag": synth_all["imag"].astype(float).to_numpy(),
                "freq": grid.astype(float)
            })
            fname = f"{prefix}_{self._pretty_uM(conc)}_{i}.txt"
            fpath = os.path.join(out_dir, fname)
            out_df.to_csv(fpath, header=False, index=False, float_format="%.10g")
            written.append(fpath)

        # preview last file
        self._show_preview(pd.read_csv(written[-1], header=None, names=["real", "imag", "freq"]))
        QMessageBox.information(self, "Done",
                                f"Generated {len(written)} file(s) in:\n{out_dir}")
