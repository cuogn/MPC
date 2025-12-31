# Bảng thông số bộ điều khiển & mô phỏng

| Nhóm | Tham số | Giá trị | Đơn vị |
|---|---|---:|---|
| Mô phỏng | dt_plant | 0.0001 | s |
| Vòng tốc độ | Ts_speed | 0.001 | s |
| Vòng dòng | i_d* | 5.0 | A |
| Vòng dòng | PI Kp | 20.0 | — |
| Vòng dòng | PI Ki | 2000.0 | — |
| Giới hạn | Vmax | 300.0 | V |
| Giới hạn | |i_q*| max | 20.0 | A |
| PID | Kp | 0.8 | — |
| PID | Ki | 10.0 | — |
| MPC | cửa sổ dự báo Np | 25 | bước |
| MPC | Q | 1.0 | — |
| MPC | R | 0.02 | — |
| MPC | Rd (phạt Δu) | 0.2 | — |

> Ghi chú: các thông số này tương ứng với default trong code hiện tại; có thể tinh chỉnh qua GUI hoặc file scenario.
