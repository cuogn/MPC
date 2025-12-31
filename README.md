# MPC IM Demo — Milestone 6

python -m venv .venv
# Windows:
.venv\Scripts\activate
# Linux/Mac:
# source .venv/bin/activate

pip install numpy pandas matplotlib scipy openpyxl PySide6 pyqtgraph

# Simple run
1. Open loop
python main.py --demo open_loop
2. Current loop
python main.py --demo current_loop
3. Speed PID
python main.py --demo speed_pid
4. Speed MPC
python main.py --demo speed_mpc
5. Compare PID vs MPC (1 kịch bản)
python main.py --demo compare_pid_mpc

# Advanced run
# Chạy “đa kịch bản” (Rs/Rr/L… + tải nặng) để ra bảng RMSE/Overshoot/Settling-time
1. Run bat
python scripts/run_batch_scenarios.py --catalog scenarios/scenarios_catalog.json
# output => outputs/batch_YYYYmmdd_HHMMSS/
1.1 metrics_tables.xlsx => bảng MPC vs PID theo từng kịch bản
1.2 metrics_all.csv, metrics_pivot.csv
1.3 csv/*.csv → log xuất từ phần mềm mô phỏng
1.4 plots/*.png + plots/*.svg → hình ω/iq*/Te/TL theo từng kịch bản

# Thiết kế mô phỏng bằng hình ảnh + quy ước trình bày
1. docs/block_diagram.png + .svg
2. docs/simulation_loop_flowchart.png + .svg
3. docs/simulation_steps.md (5–8 bullet mô tả bước chạy)
4. docs/quy_uoc_trinh_bay_mo_phong.md (chuẩn nền trắng chữ đen, PNG/SVG, dpi…)

# Bảng thông số động cơ + bộ điều khiển
1. docs/tables_motor_params.md
2. docs/tables_controller_params.md
3. configs/params_nominal.json


# Thiết kế mô phỏng bằng hình ảnh → docs/block_diagram.*, docs/simulation_loop_flowchart.*
# Bảng thông số motor + controller → docs/tables_* + configs/params_nominal.json
# Kịch bản tăng/giảm R/L + tải nặng → scenarios/scenarios_catalog.json
# Bảng RMSE/Overshoot/Settling → outputs/batch_.../metrics_tables.xlsx
# Log xuất từ phần mềm → outputs/batch_.../csv/*.csv
# Hình nền trắng chữ đen → outputs/batch_.../plots/*.png/.svg
# Thuật ngữ cửa sổ dự báo/điều khiển → trong docs và UI MPC (Np/Nc)