# MPC IM Demo — Milestone 6

GUI so sánh PID vs MPC cho điều khiển tốc độ động cơ không đồng bộ 3 pha (mô hình dq) với vòng dòng PI.

## Nâng cấp so với Milestone 5
- Chạy mô phỏng kiểu **batch**: **Run PID**, **Run MPC**, **Run Both (compare)**.
- Plot chồng PID vs MPC:
  - Speed: ω và ω*
  - Command: i_q*
- Bảng **metrics** (tính sau thời điểm step ω*): RMSE, ISE, ITAE, Overshoot, Settling time (2%), Final speed.
- **Save/Load preset** (JSON).
- **Export Bundle**: tạo `outputs/run_YYYYMMDD_HHMMSS/` chứa `config.json`, `pid.csv`, `mpc.csv`, `metrics.csv`, `plot_speed.png`, `plot_iq.png`.

## Cài đặt
```bash
pip install -r requirements.txt
```

## Chạy GUI
```bash
python main.py --gui
```
