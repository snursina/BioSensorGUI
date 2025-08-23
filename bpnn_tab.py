# --- bpnn_tab.py ---
import os, re, sys, json, copy
from pathlib import Path

import numpy as np
import pandas as pd

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QFileDialog,
    QLabel, QSpinBox, QTableView, QMessageBox, QLineEdit, QCheckBox
)
from PyQt5.QtGui import QStandardItemModel, QStandardItem
from PyQt5.QtCore import Qt

# ML deps
# (torch imports kept for backward-compat loading; not used for training now)
import torch
import torch.nn as nn
import torch.optim as optim
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.ensemble import RandomForestRegressor
import joblib


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


# ---------------------- BPNN (unused but kept) ----------------
class BPNet(nn.Module):
    def __init__(self, input_size=30, p_drop=0.1):
        super(BPNet, self).__init__()
        self.net = nn.Sequential(
            nn.Linear(input_size, 64),
            nn.ReLU(),
            nn.Dropout(p_drop),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Dropout(p_drop),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32, 1),
        )
    def forward(self, x):
        return self.net(x)
# --------------------------------------------------------------


class BPNNTab(QWidget):
    """
    Same UI/vars as before.
    Changes:
      - Uses RandomForestRegressor for training/prediction (keeps self.model, scalers, grid).
      - Global grid built from overlap (percentiles) to avoid clamping.
      - Fit scalers on TRAIN ONLY (no leakage).
      - Model saved/loaded via joblib to the same paths.
    """
    def __init__(self, parent=None):
        super().__init__(parent)

        # data holders
        self.index_rows = []      # preview rows
        self.X = None
        self.y_linear = None      # µM (original)
        self.ref_grid = None      # shared frequency grid (np.array shape (N,))
        self.input_size = 30      # will be 3*N after grid chosen

        # model/scalers
        self.model = None
        self.x_scaler = None
        self.y_scaler = None      # fits over transformed targets (log or linear)

        # save dir
        self.save_dir = Path(file_path) / ".aixsense_bpnn"
        self.save_dir.mkdir(parents=True, exist_ok=True)
        self.model_path = self.save_dir / "bp_model.pt"
        self.xs_path    = self.save_dir / "x_scaler.pkl"
        self.ys_path    = self.save_dir / "y_scaler.pkl"
        self.grid_path  = self.save_dir / "ref_grid.json"

        # --- UI ---
        root = QVBoxLayout(self)

        # row: actions
        row1 = QHBoxLayout()
        self.btnLoad = QPushButton("Load folder…")
        self.btnTrain = QPushButton("Train")
        self.btnLoadModel = QPushButton("Load model")
        self.btnPredict = QPushButton("Predict sample…")
        row1.addWidget(self.btnLoad)
        row1.addWidget(self.btnTrain)
        row1.addWidget(self.btnLoadModel)
        row1.addWidget(self.btnPredict)
        row1.addStretch(1)
        root.addLayout(row1)

        # row: config
        row2 = QHBoxLayout()
        row2.addWidget(QLabel("Target points (per spectrum):"))
        self.spnPoints = QSpinBox()
        self.spnPoints.setRange(6, 200)
        self.spnPoints.setValue(30)
        row2.addWidget(self.spnPoints)

        # row2.addWidget(QLabel("Epochs:"))
        # self.spnEpochs = QSpinBox()
        # self.spnEpochs.setRange(10, 10000)
        # self.spnEpochs.setValue(800)
        # row2.addWidget(self.spnEpochs)

        # row2.addWidget(QLabel("LR:"))
        # self.edLR = QLineEdit("2e-3")
        # self.edLR.setMaximumWidth(80)
        # row2.addWidget(self.edLR)

        row2.addWidget(QLabel("Test size:"))
        self.edTest = QLineEdit("0.2")
        self.edTest.setMaximumWidth(60)
        row2.addWidget(self.edTest)

        self.chkLogTarget = QCheckBox("Train on log10(conc + 1)")
        self.chkLogTarget.setChecked(True)
        row2.addWidget(self.chkLogTarget)

        row2.addStretch(1)
        root.addLayout(row2)

        # preview table
        self.table = QTableView()
        root.addWidget(self.table)

        # connect
        self.btnLoad.clicked.connect(self.load_folder)
        self.btnTrain.clicked.connect(self.train_model)
        self.btnLoadModel.clicked.connect(self.load_model_clicked)
        self.btnPredict.clicked.connect(self.predict_clicked)

    # -------------------- parsing helpers ---------------------

    def extract_concentration_from_filename(self, filename: str):
        base = os.path.splitext(os.path.basename(filename))[0]
        m = re.search(r'^sample_(\d+(?:[.,]\d+)?)(?:[\s_]*uM)?(?:_|$)', base, re.IGNORECASE)
        if not m:
            return None
        val = float(m.group(1).replace(",", "."))
        return val  # µM

    def _read_sample_df(self, path: str) -> pd.DataFrame:
        df = None
        try:
            df = pd.read_csv(path, header=None)
        except Exception:
            df = None
        if df is None or df.shape[1] < 3:
            try:
                df = pd.read_csv(path, header=None, sep=r"[,\s]+", engine="python")
            except Exception:
                return pd.DataFrame(columns=["real","imag","freq"])

        df = df.iloc[:, :3].copy()
        df.columns = ["real","imag","freq"]
        for c in ["real","imag","freq"]:
            df[c] = pd.to_numeric(df[c], errors="coerce")
        df = df.dropna(subset=["real","imag","freq"]).reset_index(drop=True)
        df = df.sort_values("freq").drop_duplicates(subset=["freq"]).reset_index(drop=True)
        return df

    def _build_global_grid(self, fmin_all, fmax_all, n_points: int):
        fmin_all = np.asarray(fmin_all, dtype=float)
        fmax_all = np.asarray(fmax_all, dtype=float)

        f_low_p = np.percentile(fmin_all, 90)   # robust overlap lower bound
        f_high_p = np.percentile(fmax_all, 10)  # robust overlap upper bound

        if not np.isfinite(f_low_p) or not np.isfinite(f_high_p):
            return None

        if f_high_p <= f_low_p:
            f_low_p  = float(np.max(fmin_all))
            f_high_p = float(np.min(fmax_all))
            if f_high_p <= f_low_p:
                return None

        if f_low_p > 0 and (f_high_p / f_low_p) > 1.5:
            grid = np.logspace(np.log10(f_low_p), np.log10(f_high_p), n_points)
        else:
            grid = np.linspace(f_low_p, f_high_p, n_points)
        return grid.astype(float)
    

    def _interp_to_grid_clamped(self, df: pd.DataFrame, grid: np.ndarray):
        f = df["freq"].to_numpy(float)
        r = df["real"].to_numpy(float)
        i = df["imag"].to_numpy(float)
        real_g = np.interp(grid, f, r)
        imag_g = np.interp(grid, f, i)
        return real_g, imag_g

    def _flatten_features(self, real_g, imag_g, grid):
        return np.column_stack([real_g, imag_g, grid]).reshape(-1)

    # ----------------------- UI actions -----------------------

    def load_folder(self):
        folder = QFileDialog.getExistingDirectory(self, "Select data folder", "")
        if not folder:
            return

        n_points = int(self.spnPoints.value())

        files = sorted([os.path.join(folder, f) for f in os.listdir(folder) if f.lower().endswith(".txt")])
        if not files:
            QMessageBox.warning(self, "No files", "No .txt files found in the selected folder.")
            return

        dfs, concs, names, fmins, fmaxs = [], [], [], [], []
        skipped_no_conc = 0
        skipped_short = 0
        self.index_rows.clear()

        for p in files:
            y = self.extract_concentration_from_filename(os.path.basename(p))
            if y is None:
                skipped_no_conc += 1
                continue
            df = self._read_sample_df(p)
            if len(df) < 2:
                skipped_short += 1
                continue
            dfs.append(df); concs.append(float(y)); names.append(os.path.basename(p))
            fmins.append(df["freq"].min()); fmaxs.append(df["freq"].max())
            self.index_rows.append({"file": os.path.basename(p), "rows": len(df), "y_uM": y})

        if not dfs:
            QMessageBox.warning(self, "No data", "No valid files with parsable concentration and >= 2 rows.")
            return

        grid = self._build_global_grid(np.array(fmins), np.array(fmaxs), n_points)
        if grid is None:
            QMessageBox.warning(self, "Grid error", "Could not build a global frequency grid from the dataset.")
            return

        X_list = []
        for df in dfs:
            r_g, i_g = self._interp_to_grid_clamped(df, grid)
            vec = self._flatten_features(r_g, i_g, grid)
            X_list.append(vec)

        self.X = np.vstack(X_list).astype(float)
        self.y_linear = np.array(concs, dtype=float).reshape(-1, 1)  # µM for metrics later
        self.ref_grid = grid.astype(float)
        self.input_size = self.X.shape[1]

        # preview
        model = QStandardItemModel(self)
        model.setColumnCount(3)
        model.setHorizontalHeaderLabels(["file", "rows", "y_uM"])
        for nm, df, y in zip(names, dfs, concs):
            model.appendRow([
                QStandardItem(str(nm)),
                QStandardItem(str(len(df))),
                QStandardItem(f"{y:.6g}")
            ])
        self.table.setModel(model)
        self.table.resizeColumnsToContents()

        QMessageBox.information(self, "Loaded",
            f"Built global grid with N={n_points} points\n"
            f"Usable files: {len(self.X)} / {len(files)}\n"
            f"Input size: {self.input_size} features (3×N)")

    def _try_load_model_files(self):
        """
        Load model + scalers + grid from disk.
        Now prefers joblib (RandomForest); falls back to torch for legacy .pt.
        """
        if not (self.xs_path.exists() and self.ys_path.exists() and self.grid_path.exists() and self.model_path.exists()):
            return False
        try:
            meta = json.loads(self.grid_path.read_text())
            grid = np.array(meta["grid"], dtype=float)
            in_size = int(meta["input_size"])
            use_log = bool(meta.get("use_log_target", True))

            # Try joblib (RF) first
            try:
                model = joblib.load(self.model_path)
                # minimal check: RF has predict attribute
                _ = getattr(model, "predict")
            except Exception:
                # Fallback: legacy torch model (won't be used after RF retrain)
                model = BPNet(input_size=in_size)
                model.load_state_dict(torch.load(str(self.model_path), map_location="cpu"))
                model.eval()

            xs = joblib.load(self.xs_path)
            ys = joblib.load(self.ys_path)

            self.model = model
            self.x_scaler = xs
            self.y_scaler = ys
            self.ref_grid = grid
            self.input_size = in_size
            self.chkLogTarget.setChecked(use_log)
            return True
        except Exception as e:
            print("Load model failed:", e)
            return False

    def load_model_clicked(self):
        if self._try_load_model_files():
            QMessageBox.information(self, "Model", f"Loaded model, scalers, and grid from:\n{self.save_dir}")
        else:
            QMessageBox.warning(self, "Model", "No saved model+grid found, or load failed.")

    # --------------------- training / eval --------------------

    def _transform_y(self, y_linear: np.ndarray, use_log: bool):
        """Return (y_transformed, inverse_fn)."""
        if use_log:
            y_t = np.log10(np.maximum(0.0, y_linear) + 1.0)
            def inv_fn(y_t_unscaled):
                return np.maximum(0.0, (10.0 ** y_t_unscaled) - 1.0)
            return y_t, inv_fn
        else:
            def inv_fn(y_unscaled): return y_unscaled
            return y_linear, inv_fn

    def train_model(self):
        if self.X is None or self.y_linear is None or self.ref_grid is None:
            QMessageBox.information(self, "Load data", "Load a folder first (to build the global grid).")
            return

        # parse hyperparams (kept for UI compatibility; RF ignores epochs/LR)
        try:
            epochs = 100#int(self.spnEpochs.value())
            lr = 0.2#float(eval(self.edLR.text(), {}, {}))   # unused by RF
            test_size = float(self.edTest.text())
            use_log = bool(self.chkLogTarget.isChecked())
        except Exception:
            QMessageBox.warning(self, "Config", "Invalid training parameters.")
            return

        # ----- targets transform -----
        y_t, inv_y = self._transform_y(self.y_linear, use_log=use_log)

        # ----- split BEFORE fitting scalers (to prevent leakage) -----
        X_train_raw, X_val_raw, y_train_raw, y_val_raw = train_test_split(
            self.X, y_t, test_size=test_size, random_state=42
        )

        # ----- fit scalers on TRAIN ONLY, then transform both -----
        x_scaler = StandardScaler().fit(X_train_raw)
        X_train = x_scaler.transform(X_train_raw)
        X_val   = x_scaler.transform(X_val_raw)

        y_scaler = StandardScaler().fit(y_train_raw)
        y_train = y_scaler.transform(y_train_raw).ravel()
        y_val   = y_scaler.transform(y_val_raw).ravel()

        # ----- RandomForestRegressor -----
        model = RandomForestRegressor(
            n_estimators=400,
            max_depth=None,
            random_state=42,
            n_jobs=-1
        )
        model.fit(X_train, y_train)

        # evaluation on validation set in µM
        y_val_pred_scaled = model.predict(X_val).reshape(-1, 1)
        y_val_pred_t = y_scaler.inverse_transform(y_val_pred_scaled)
        y_val_true_t = y_scaler.inverse_transform(y_val.reshape(-1, 1))

        y_val_pred = inv_y(y_val_pred_t)
        y_val_true = inv_y(y_val_true_t)

        mae = float(np.mean(np.abs(y_val_true.flatten() - y_val_pred.flatten())))
        rmse = float(np.sqrt(np.mean((y_val_true.flatten() - y_val_pred.flatten())**2)))

        # save artifacts + grid + settings (use joblib for model)
        saved = False
        try:
            joblib.dump(model, str(self.model_path))
            joblib.dump(x_scaler, str(self.xs_path))
            joblib.dump(y_scaler, str(self.ys_path))
            meta = {
                "grid": self.ref_grid.tolist(),
                "input_size": int(self.input_size),
                "use_log_target": bool(use_log),
                "note": "overlap grid; features are [real,imag,freq] per point; RF model"
            }
            self.grid_path.write_text(json.dumps(meta))
            saved = True
        except Exception as e:
            print("Save failed:", e)

        # stash for session
        self.model = model
        self.x_scaler = x_scaler
        self.y_scaler = y_scaler

        msg = (f"Trained RF on {len(X_train)} / tested on {len(X_val)} samples.\n"
               f"MAE: {mae:.3f} µM   RMSE: {rmse:.3f} µM\n"
               f"Grid: {len(self.ref_grid)} points, [{self.ref_grid.min():.6g} .. {self.ref_grid.max():.6g}]")
        if saved:
            msg += f"\nSaved model + scalers + grid to:\n{self.save_dir}"
        QMessageBox.information(self, "Training complete", msg)

        # --- Diagnostics: per-concentration RMSE/MAE and CSV export ---
        y_val_true_um = y_val_true.flatten()
        y_val_pred_um = y_val_pred.flatten()

        # Per unique concentration (rounded to 1 µM just in case)
        import numpy as _np
        def _rmse(a,b): return float(_np.sqrt(_np.mean((a-b)**2)))
        def _mae(a,b):  return float(_np.mean(_np.abs(a-b)))

        vals = []
        for c in sorted(_np.unique(_np.round(y_val_true_um, 6))):
            m = (abs(y_val_true_um - c) < 1e-6)
            if m.sum() >= 1:
                vals.append({
                    "conc_uM": float(c),
                    "n": int(m.sum()),
                    "rmse_uM": _rmse(y_val_true_um[m], y_val_pred_um[m]),
                    "mae_uM":  _mae (y_val_true_um[m], y_val_pred_um[m]),
                })

        # Save validation predictions for a quick look
        import pandas as _pd
        pred_df = _pd.DataFrame({
            "y_true_uM": y_val_true_um,
            "y_pred_uM": y_val_pred_um
        })
        pred_csv_path = str(self.save_dir / "val_predictions.csv")
        pred_df.to_csv(pred_csv_path, index=False)

        # Add brief per-class summary to the message
        per_class_summary = "\n".join([f"  {v['conc_uM']:.0f} µM  (n={v['n']}):  RMSE {v['rmse_uM']:.1f}  MAE {v['mae_uM']:.1f}" for v in vals])
        msg += f"\n\nPer-concentration validation:\n{per_class_summary}\nSaved validation preds: {pred_csv_path}"


    # ----------------------- prediction -----------------------

    def predict_single(self, path: str):
        if self.ref_grid is None or self.model is None or self.x_scaler is None or self.y_scaler is None:
            if not self._try_load_model_files():
                QMessageBox.information(self, "Model", "Train or load a model first.")
                return None

        # read & vectorize
        df = self._read_sample_df(path)
        if len(df) < 2:
            QMessageBox.warning(self, "Predict", "Sample has fewer than 2 valid rows.")
            return None
        r_g, i_g = self._interp_to_grid_clamped(df, self.ref_grid)
        vec = self._flatten_features(r_g, i_g, self.ref_grid)

        X_sample = np.array(vec, dtype=float).reshape(1, -1)
        X_scaled = self.x_scaler.transform(X_sample)

        # infer (RF)
        y_pred_scaled = self.model.predict(X_scaled).reshape(-1, 1)
        y_pred_t = self.y_scaler.inverse_transform(y_pred_scaled)

        # invert log if needed
        use_log = bool(self.chkLogTarget.isChecked())
        if use_log:
            y_pred = np.maximum(0.0, (10.0 ** y_pred_t) - 1.0)
        else:
            y_pred = y_pred_t
        return float(y_pred[0][0])

    def predict_clicked(self):
        sample_path, _ = QFileDialog.getOpenFileName(
            self, "Select sample file", "", "Text files (*.txt);;All files (*)"
        )
        if not sample_path:
            return
        pred = self.predict_single(sample_path)
        if pred is None:
            return
        QMessageBox.information(self, "Prediction", f"Predicted concentration:\n{pred:.2f} µM")
