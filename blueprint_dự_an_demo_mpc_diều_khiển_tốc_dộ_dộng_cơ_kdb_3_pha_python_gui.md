# 1) Mục tiêu đề tài (Scope rõ ràng)

## 1.1 Mục tiêu kỹ thuật
- Mô phỏng **động cơ không đồng bộ 3 pha** bằng mô hình **dq (continuous-time)** + phương trình cơ học.
- Thiết kế **vòng tốc độ** gồm 2 phương án để so sánh công bằng:
  - **MPC (Model Predictive Control)**: tối ưu để bám tốc độ và xử lý ràng buộc.
  - **PID tốc độ**: baseline.
- Giữ cấu trúc truyền động “kinh điển và dễ demo”:
  - Vòng tốc độ (MPC/PID) → tạo **i_q\*** (hoặc T_e\*)
  - Vòng dòng (PI) → tạo **v_d, v_q** (có bão hòa theo Vdc)
  - Plant: mô hình dq của IM + cơ học.

## 1.2 Mục tiêu demo (sản phẩm)
- Ứng dụng Python có **GUI** cho phép:
  - Chọn chế độ điều khiển: **MPC vs PID** (toggle).
  - Chỉnh tham số: motor, tải, giới hạn dòng/điện áp, horizon MPC, trọng số Q/R…
  - Chạy mô phỏng và xem **đồ thị realtime**: ω, ω\*, i_d/i_q, v_d/v_q, T_e, T_L.
  - Chạy các kịch bản: step tốc độ, ramp tốc độ, step tải, thay đổi tham số.
  - Xuất kết quả: CSV + ảnh đồ thị + bảng chỉ số (RMSE/ISE/ITAE/overshoot).

## 1.3 Tiêu chí “done”
- MPC bám tốc độ tốt hơn hoặc xử lý bão hòa tốt hơn PID trong ít nhất 2 kịch bản.
- Có dashboard GUI chạy ổn, không giật, mô phỏng trong thời gian hợp lý.
- Có report ngắn (markdown/pdf) mô tả mô hình + thuật toán + kết quả.

---

# 2) Kiến trúc hệ thống mô phỏng tổng thể

## 2.1 Sơ đồ khối
- Reference generator: ω\*(t) (step/ramp/chuỗi đoạn)
- Disturbance: T_L(t)
- Speed controller (select):
  - PID: e = ω\* − ω → i_q\* (có bão hòa)
  - MPC: solve QP → i_q\* (có bão hòa + penalize Δi_q\*)
- Current controller (FOC PI):
  - i_d\* cố định để giữ từ thông (field)
  - i_q\* từ vòng tốc độ
  - PI_d/PI_q → v_d, v_q + anti-windup
- Inverter model (đơn giản): saturation theo Vdc hoặc |v| ≤ Vmax
- Plant:
  - IM dq model (dòng/từ thông) + cơ học ω
  - Option: abc↔dq có thể bỏ qua nếu bạn mô phỏng trực tiếp dq
- Logging + metrics
- GUI: điều khiển chạy/dừng, chỉnh tham số, plot realtime

## 2.2 Quyết định “chốt” cho demo (để khả thi)
- MPC **không điều khiển trực tiếp v_d, v_q** (khó vì động học điện nhanh, nặng tính toán).
- MPC sẽ là **vòng tốc độ**, ra **i_q\***.
- Plant vẫn mô phỏng đầy đủ dq (để “đúng đề tài”), nhưng MPC dùng mô hình dự đoán tốc độ bậc 1 (có thể xem như tuyến tính hóa của phần cơ học).

---

# 3) Mô hình động cơ KĐB trong hệ dq (continuous-time)

> Gợi ý dùng mô hình “stator current + rotor flux” phổ biến cho FOC.

## 3.1 Trạng thái tối thiểu
Chọn:
- x = [ i_ds, i_qs, ψ_dr, ψ_qr, ω ]ᵀ
- Nhiễu/ngoại lực: d = T_L
- Đầu vào plant: u = [ v_ds, v_qs ]ᵀ
- Đầu ra đo: y = [ ω, i_ds, i_qs ]

## 3.2 Tham số motor
- R_s, R_r, L_s, L_r, L_m
- σ = 1 − L_m²/(L_s L_r)
- L_σs = σ L_s
- τ_r = L_r / R_r
- p: số đôi cực
- J, B

## 3.3 Phương trình cơ học
- ω̇ = (T_e − T_L − B ω)/J

## 3.4 Mô men điện từ (FOC-friendly)
- T_e = (3/2) p (L_m/L_r) ( ψ_dr i_qs − ψ_qr i_ds )

## 3.5 Phương trình điện (dạng chuẩn, dùng trong nhiều giáo trình)
Viết dưới dạng:
- i̇_s = f(i_s, ψ_r, ω, v_s)
- ψ̇_r = f(i_s, ψ_r, ω)

Trong triển khai demo, bạn có thể dùng bộ phương trình chuẩn sau (được dùng phổ biến cho mô phỏng FOC IM):
- ψ̇_dr = (L_m/τ_r) i_ds − (1/τ_r) ψ_dr + ω_sl ψ_qr
- ψ̇_qr = (L_m/τ_r) i_qs − (1/τ_r) ψ_qr − ω_sl ψ_dr
- i̇_ds = (1/L_σs) [ v_ds − R_s i_ds − (L_m/L_r) ψ̇_dr + ω_e L_σs i_qs ]
- i̇_qs = (1/L_σs) [ v_qs − R_s i_qs − (L_m/L_r) ψ̇_qr − ω_e L_σs i_ds ]

Trong đó:
- ω_e = ω_s (tốc độ điện đồng bộ của hệ dq)
- ω_sl là slip (trong FOC có thể tính từ i_q và ψ_dr: ω_sl = (L_m/τ_r) i_qs / ψ_dr ) với ψ_dr>0

> Lưu ý: Có nhiều dạng tương đương. Mục tiêu demo là “đúng cấu trúc + chạy ổn”. Khi code, bạn sẽ chuẩn hóa lại để tránh mâu thuẫn ký hiệu.

---

# 4) Rời rạc hóa và mô phỏng thời gian

## 4.1 Multi-rate (khuyến nghị)
- Ts_e (điện): 50–200 µs (đủ mịn để dòng ổn)
- Ts_s (tốc độ / MPC / PID): 1–5 ms

Mỗi bước Ts_s:
- Cập nhật MPC/PID → i_q\*
- Chạy N = Ts_s/Ts_e vòng nội (PI dòng + plant RK4)

## 4.2 Tích phân số
- Dùng RK4 cho plant (ổn định hơn Euler).

---

# 5) Thiết kế PID tốc độ (baseline)

## 5.1 Dạng PID
- i_q\* = sat( Kp e + Ki ∫e dt + Kd de/dt )
- Có anti-windup theo saturator.

## 5.2 i_d\* (flux)
- Chọn i_d\* hằng (ví dụ để tạo ψ_r nominal).
- Hoặc có ramp khi start.

---

# 6) Thiết kế MPC vòng tốc độ (QP tuyến tính, chạy nhanh)

## 6.1 Mô hình dự đoán cho MPC
Để MPC nhẹ nhưng hợp lý, dùng mô hình cơ học tuyến tính với đầu vào i_q\*:
- ω̇ = a ω + b i_q\* + g T_L
Trong đó (xấp xỉ theo FOC):
- a = −B/J
- b = K_t/J, với K_t ≈ (3/2) p (L_m/L_r) ψ_dr_nom
- g = −1/J

Rời rạc ZOH hoặc Euler:
- ω_{k+1} = A ω_k + B i_{q,k} + G T_{L,k}

## 6.2 Bài toán tối ưu
Trong horizon Np, Nc:
Minimize:
- Σ (ω_ref(k+i) − ω(k+i))² * Q
- + Σ (i_q(k+i))² * R
- + Σ (Δi_q(k+i))² * Rd

Constraints:
- |i_q| ≤ i_q_max
- |Δi_q| ≤ diq_max (tùy chọn)

## 6.3 Solver
- Dùng **cvxpy** + **OSQP** (QP nhanh, dễ cài).
- Để realtime mượt: dựng bài toán dạng parametric (Parameter) và warm-start.

---

# 7) Kịch bản mô phỏng & tiêu chí đánh giá

## 7.1 Kịch bản tối thiểu
1) Step tốc độ: 0 → 1500 rpm, T_L = 0
2) Step tải: giữ tốc độ, T_L: 0 → T_nom (tại t=1.5s)
3) Ramp tốc độ: 0 → 1500 rpm trong 2s
4) Sai lệch tham số: tăng R_r hoặc J 20% (robustness)

## 7.2 Chỉ số
- RMSE(ω)
- ISE, ITAE
- Overshoot, settling time
- Control effort: Σ i_q², Σ Δi_q²
- Tỷ lệ thời gian bão hòa (i_q hoặc v)

---

# 8) GUI: yêu cầu chức năng & bố cục

## 8.1 Công nghệ GUI
- Khuyến nghị: **PySide6 (Qt)** + **pyqtgraph** (plot realtime rất ổn)

## 8.2 Màn hình chính
- Panel trái (Inputs):
  - Dropdown: Controller = {MPC, PID}
  - ω\* profile: step/ramp/custom
  - T_L profile: step/custom
  - Tham số giới hạn: i_q_max, Vdc/Vmax
  - MPC params: Np, Nc, Q, R, Rd
  - PID params: Kp, Ki, Kd
  - Buttons: Start / Pause / Reset / Export
- Panel phải (Plots realtime):
  - Plot1: ω và ω\*
  - Plot2: i_d, i_q và i_d\*, i_q\*
  - Plot3: v_d, v_q (bão hòa)
  - Plot4: T_e và T_L
- Tab “Metrics”: bảng chỉ số + so sánh MPC vs PID

## 8.3 Kiến trúc chạy realtime
- Simulation chạy trong QThread/Worker.
- GUI nhận data qua signal, cập nhật plot theo decimation (ví dụ 50–100 Hz).

---

# 9) Cấu trúc thư mục dự án (chuẩn hoá ngay từ đầu)

```
mpc_im_demo/
  README.md
  requirements.txt
  main.py
  app/
    gui/
      main_window.py
      plots.py
      controllers_panel.py
    sim/
      motor_im_dq.py
      foc_current_loop.py
      scenarios.py
      simulator.py
      logger.py
    control/
      pid_speed.py
      mpc_speed.py
      estimators.py   # optional: load torque/flux estimation
    metrics/
      metrics.py
    utils/
      units.py
      saturation.py
      signals.py
  tests/
    test_discretization.py
    test_mpc_qp.py
  outputs/
    ...
```

---

# 10) Lộ trình triển khai (từ 0 → demo)

## Milestone 1 — “Chạy plant đúng” (không điều khiển)
- Code motor_im_dq.py + RK4
- Test: áp điện áp cố định → dòng và ω biến thiên hợp lý

## Milestone 2 — “FOC + PI dòng”
- Implement PI_d/PI_q, bão hòa v_d/v_q, anti-windup
- Test: giữ i_d, i_q theo setpoint (tracking)

## Milestone 3 — “PID tốc độ”
- PID tốc độ → i_q\*
- Test: step tốc độ + step tải

## Milestone 4 — “MPC tốc độ”
- Implement mpc_speed.py (cvxpy + OSQP)
- Test: so sánh MPC vs PID cùng điều kiện ràng buộc

## Milestone 5 — “GUI realtime”
- Build GUI tối thiểu (Start/Stop + plot ω)
- Sau đó thêm panel tham số + export

## Milestone 6 — “Đánh giá + report”
- Metrics + bảng kết quả
- Viết README + hình minh hoạ

---

# 11) Kết quả mong muốn (để bám sát khi thực thi)

- Một app chạy được trên máy cá nhân:
  - Bật lên → chọn kịch bản → chọn MPC/PID → chạy → thấy đồ thị.
  - Xuất CSV/ảnh/metrics.
- MPC thể hiện lợi thế rõ khi có bão hòa hoặc thay đổi tải.
- Code có cấu trúc sạch, dễ mở rộng (thêm NMPC hoặc FCS-MPC sau).

---

# 12) Bộ công nghệ đề xuất (requirements)
- numpy, scipy
- cvxpy, osqp
- PySide6, pyqtgraph
- pandas (export/metrics)

> Khi bắt đầu thực thi, ta sẽ chốt các giá trị tham số motor mẫu và set mặc định trong GUI.

