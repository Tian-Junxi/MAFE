#!/usr/bin/env bash
# 从 Excel 一键训练共享 GPR 模型。
# 用法：
#   bash scripts/train_from_xlsx.sh your_data.xlsx
#   bash scripts/train_from_xlsx.sh your_data.xlsx ./gpr_model_result
#   bash scripts/train_from_xlsx.sh your_data.xlsx ./gpr_model_result config/train_params_excel_current.yaml
set -e

if [ $# -lt 1 ]; then
  echo "Usage: bash train_from_xlsx.sh input.xlsx [model_dir] [param_file] [sheet]"
  exit 1
fi

XLSX_PATH="$1"
MODEL_DIR="${2:-${HOME}/catkin_ws/output}"
PKG_DIR="$(rospack find gpr_shared_frequency_model)"
PARAM_FILE="${3:-${PKG_DIR}/config/train_params_excel_current.yaml}"
SHEET_SELECTOR="${4:-0}"
TMP_CSV="/tmp/gpr_shared_frequency_model/from_excel.csv"

mkdir -p /tmp/gpr_shared_frequency_model
mkdir -p "${MODEL_DIR}"

python3 "${PKG_DIR}/scripts/xlsx_to_csv.py" "${XLSX_PATH}" "${TMP_CSV}" "${SHEET_SELECTOR}"

roslaunch gpr_shared_frequency_model train_shared_gpr.launch \
  csv_path:="${TMP_CSV}" \
  output_dir:="${MODEL_DIR}" \
  param_file:="${PARAM_FILE}"

echo ""
echo "Training finished. Model files are in: ${MODEL_DIR}"
echo "  ${MODEL_DIR}/gyro_gpr_model.gpr"
echo "  ${MODEL_DIR}/acc_gpr_model.gpr"
