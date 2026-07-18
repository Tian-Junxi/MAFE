# gpr_shared_frequency_model

> 📄 [中文版 (Chinese)](README_zh.md)

A ROS1 / Catkin C++ package for training a **four-motor shared GPR dominant frequency model** from an experimental database, and predicting vibration noise dominant frequencies and variances given four motor speeds at runtime.

**Core input** from Excel/CSV database:
- Various thrust and attitude angle conditions;
- Four motor speeds: `m1, m2, m3, m4`;
- 1–4 vibration noise dominant frequencies obtained via `FFT + Gaussian peak fitting`;
- Gaussian fitting variance for each dominant frequency.

**Core output:**
```
output_dir/gyro_gpr_model.gpr
output_dir/acc_gpr_model.gpr
```

These `.gpr` files are the trained model files, loaded by the prediction node at inference time.

---

## 1. Methodology

This package implements:

```
Mean-speed prior + Iterative re-matching + Four-motor shared GPR
```

Instead of training four separate models (one per motor), a single equivalent single-motor model is shared:

$$f_d = g(m)$$

where $m$ is any motor speed and $f_d$ is the corresponding vibration dominant frequency.

Given four motor speeds `[m1, m2, m3, m4]`, the prediction node calls the same GPR model four times, outputting:

```
[mean1, var1, mean2, var2, mean3, var3, mean4, var4]
```

where `mean_i` is the predicted dominant frequency for motor $i$, and `var_i` is the predicted variance.

---

## 2. Build

```bash
cd ~/catkin_ws/src
cp -r gpr_shared_frequency_model .
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

**Dependencies:**

```bash
sudo apt-get install ros-${ROS_DISTRO}-roscpp ros-${ROS_DISTRO}-std-msgs libeigen3-dev libyaml-cpp-dev
```

---

## 3. Excel to CSV Conversion

The C++ training node reads CSV. To avoid heavy dependencies for parsing `.xlsx` directly in ROS/C++, this package includes a zero-dependency Excel-to-CSV script:

```bash
python3 $(rospack find gpr_shared_frequency_model)/scripts/xlsx_to_csv.py \
  your_database.xlsx \
  /tmp/gpr_shared_frequency_model/from_excel.csv
```

This script preserves empty columns from Excel, so YAML column indices still correspond to Excel's original column numbers (0-based):

```
A -> 0, B -> 1, C -> 2, ...
```

A one-click training script is also provided:

```bash
bash $(rospack find gpr_shared_frequency_model)/scripts/train_from_xlsx.sh \
  your_database.xlsx \
  ./gpr_shared_frequency_model_result
```

---

## 4. Training with the Example Database

The package includes an example CSV converted from an uploaded Excel:

```
examples/updated_database_from_uploaded_excel.csv
```

To train directly:

```bash
cd ~/catkin_ws
source devel/setup.bash

roslaunch gpr_shared_frequency_model train_shared_gpr.launch
```

This uses:
- `examples/updated_database_from_uploaded_excel.csv`
- `config/train_params_excel_current.yaml`

Output directory: `~/catkin_ws/gpr_shared_frequency_model_result/`

**Output files:**

| File | Description |
|------|-------------|
| `gyro_gpr_model.gpr` | Gyroscope shared GPR dominant frequency model |
| `acc_gpr_model.gpr` | Accelerometer shared GPR dominant frequency model |
| `*_iter_stats.csv` | Per-iteration R², RMSE, and match-change row count |
| `*_final_matched_samples.csv` | Final expanded `(motor_speed, matched_frequency)` training samples |

---

## 5. Training with Your Own Excel Data

Assuming your Excel file is at `/home/tjx/data/new_database.xlsx`:

```bash
cd ~/catkin_ws
source devel/setup.bash

bash $(rospack find gpr_shared_frequency_model)/scripts/train_from_xlsx.sh \
  /home/tjx/data/new_database.xlsx \
  ./gpr_shared_frequency_model_result \
  $(rospack find gpr_shared_frequency_model)/config/train_params_excel_current.yaml
```

---

## 6. Excel Column Configuration

`config/train_params_excel_current.yaml` corresponds to the following Excel layout:

| Column | Content |
|--------|---------|
| A | Thrust or attitude angle label |
| C–F | Motor 1, Motor 2, Motor 3, Motor 4 speeds |
| H–J | Gyroscope dominant frequency & variance 1–3 |
| L–N | Gyroscope dominant frequency 1–3 |
| Q–S | Accelerometer dominant frequency & variance 1–3 |
| U–W | Accelerometer dominant frequency 1–3 |

---

## 7. Prediction

After training, run the prediction node:

```bash
roslaunch gpr_shared_frequency_model predict_shared_gpr.launch
```

This loads the `.gpr` models and subscribes to motor speed topics to publish predicted vibration frequencies in real time.
