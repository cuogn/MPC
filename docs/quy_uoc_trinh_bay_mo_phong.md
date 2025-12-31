# Quy ước trình bày mô phỏng

## Thuật ngữ
- Cửa sổ dự báo (Np): số bước tương lai mà MPC dùng để dự đoán đáp ứng.
- Cửa sổ điều khiển (Nc): số bước điều khiển được tối ưu (nếu dùng Nc < Np).
- ω: tốc độ góc rotor (rad/s). Trong báo cáo thường quy đổi sang rpm để trực quan.
- ω*: tốc độ đặt (reference).
- T_L: mô-men tải (N·m), mô phỏng lực cản tác động lên trục.
- i_q*: lệnh dòng q (A), tỷ lệ trực tiếp với mô-men điện từ trong FOC.

## Chuẩn đồ thị/hình mô phỏng
- Nền trắng, chữ đen (axes/labels/ticks).
- Lưới (grid) mờ, line rõ ràng, font thống nhất.
- Xuất hình: PNG + SVG, độ phân giải 300 dpi.
- Tên file hình có quy ước: `S01_compare.png`, `S02_compare.png`, ...

## Template xuất hình (Python/Matplotlib)
Trong project đã có module: `app/plotting/plot_style.py` gồm:
- `apply_thesis_style()`
- `save_figure(fig, path_no_ext, dpi=300)`
