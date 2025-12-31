# Các bước thực hiện mô phỏng (tóm tắt)

1. Khởi tạo tham số động cơ IM (Rs, Rr, Ls, Lr, Lm, p, J, B) và trạng thái ban đầu.
2. Tạo kịch bản: ω\*(t) và T_L(t) (step/ramp/sin/random hoặc timeline).
3. Vòng lặp theo thời gian (chu kỳ Ts_speed):
   - Đọc ω (và các trạng thái điện nếu cần).
   - Tính sai lệch tốc độ e = ω\* − ω.
   - **PID/MPC vòng tốc độ** sinh lệnh **i_q\*** (có ràng buộc |i_q\*| ≤ i_q,max).
   - **Vòng dòng PI + FOC** tính ra điện áp **v_d, v_q**.
   - Cập nhật plant IM dq + phương trình cơ bằng tích phân số.
   - Log dữ liệu: ω, ω\*, i_d/i_q, i_q\*, v_d/v_q, T_e, T_L.
4. Sau mô phỏng: tính chỉ tiêu RMSE, overshoot, settling time… và xuất bảng so sánh.
