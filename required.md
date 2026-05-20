Objective: Hãy viết mã nguồn Python sử dụng thư viện PySide6 để tạo ra một giao diện Dashboard điều khiển động cơ (IM Speed Control Research Dashboard) giống như mô tả cấu trúc dưới đây. Sử dụng PyQtGraph để vẽ các đồ thị thời gian thực và QSS (Qt Style Sheets) để tùy biến giao diện tối (Dark Mode).

1. CẤU TRÚC LAYOUT CHÍNH (MAIN LAYOUT)
Cửa sổ chính: QMainWindow làm nền.

Widget trung tâm: QWidget với QVBoxLayout tổng.

Bố cục gồm 2 tầng chính:

Tầng 1 (Top): Header bar (QHBoxLayout).

Tầng 2 (Bottom): Workspace (QHBoxLayout) chia làm 3 cột chính: Cột trái (Sidebar 20%), Cột giữa (Main Content 60%), Cột phải (Sidebar 20%).

2. CHI TIẾT CÁC THÀNH PHẦN (WIDGET COMPONENT TREE)
A. Header Bar (QHBoxLayout)
Trái: QLabel chứa icon (sử dụng ký tự Unicode hoặc icon mặc định) + QLabel tiêu đề chính "IM Speed Control Research Dashboard" (Font size lớn, Bold) + QLabel phụ màu xám "Comparison of PID and SFMPC for 3 Phase Induction Motor".

Giữa: Một QWidget làm thanh chọn chế độ. Sử dụng QButtonGroup chứa 3 QPushButton nằm ngang: PID Only, SFMPC Only, Compare (PID vs SFMPC). Nút thứ 3 được set check mặc định và có style màu nền xanh lá cây.

Phải: QHBoxLayout chứa:

Trạng thái: Chấm tròn xanh lá (vẽ bằng QSS border-radius) + QLabel chữ "Status: RUNNING" màu xanh lá.

Thời gian: QLabel chữ "Sim. Time: 2.45 s".

B. Cột Trái - Bảng Điều Khiển (QVBoxLayout đặt trong QScrollArea)
Mỗi vùng là một QGroupBox có tiêu đề viết hoa, viền mờ màu xanh dương:

Simulation Control (QGroupBox):

Hàng 1: 3 QPushButton: Start (nền xanh lá), Stop (nền đỏ), Reset (nền xám).

Hàng 2: 3 QPushButton dạng outline/border: Load Config, Save Config, Export Data.

Test Scenario (QGroupBox): Layout dạng QFormLayout hoặc QGridLayout gồm các cặp nhãn và ô nhập:

Reference Speed (Sử dụng QSpinBox hoặc QLineEdit, giá trị 1500, hậu tố rpm).

Speed Step Time (QDoubleSpinBox, giá trị 0.30, hậu tố s).

Load Torque (QDoubleSpinBox, giá trị 5.00, hậu tố N·m).

Disturbance: QCheckBox hoặc một widget Custom Toggle Switch (đang ở trạng thái ON).

Parameter Mismatch: Custom Toggle Switch (trạng thái OFF).

PID Parameters (QGroupBox): QFormLayout gồm 3 ô QDoubleSpinBox: Kp (0.800), Ki (10.000), Kd (0.000).

SFMPC Parameters (QGroupBox): Các ô QSpinBox và QDoubleSpinBox: Prediction Horizon (Nc) (10), Control Horizon (Nu) (3), Sampling Time (Ts) (0.0010 s), λ (1.00), μ (0.10), Switching Penalty (0.01).

Motor & Simulation (QGroupBox):

Motor Model: QComboBox với giá trị chọn là "IM - 2.2kW".

Sampling Time (0.0010 s), Noise Level (0.0004).

Dưới cùng cột trái: QLabel hiển thị trạng thái hệ thống: "Ready" kèm icon tick xanh.

C. Cột Giữa - Vùng Hiển Thị Chính (QVBoxLayout)
Summary Indicators (QGroupBox): Bố cục QHBoxLayout chứa 5 thẻ chỉ số (QWidget custom). Mỗi thẻ gồm:

Tên chỉ số (QLabel phía trên).

Giá trị so sánh (QHBoxLayout chứa giá trị PID màu cam và SFMPC màu xanh cyan).

Phần trăm cải thiện (QLabel màu xanh lá phía dưới kèm mũi tên đi xuống ▼).

Dữ liệu khởi tạo: - RMSE (rpm): PID 18.4 | SFMPC 6.5 (▼ 64.7%)

Overshoot (%): PID 12.4 | SFMPC 3.2 (▼ 74.2%)

Settling Time (s): PID 0.24 | SFMPC 0.55 (▼ 56.3%)

Steady-State Error (rpm): PID 7.5 | SFMPC 0.3 (▼ 96.0%)

Rise Time (s): PID 0.29 | SFMPC 0.15 (▼ 48.3%)

Realtime Monitoring (Đồ thị - QVBoxLayout):

Tích hợp 3 widget đồ thị độc lập từ thư viện pyqtgraph (pg.PlotWidget()), đặt nền màu tối đồng bộ với QSS.

Đồ thị 1: Tiêu đề "Speed Reference vs Actual Speed". Trục Y: 0 - 1800. Gồm 3 đường: Reference (đứt nét trắng), PID (đường cam), SFMPC (đường cyan).

Đồ thị 2: Tiêu đề "Tracking Error (Speed Error)". Trục Y: -60 đến 60. Gồm 2 đường sai số và 2 đường giới hạn đứt nét trắng tại vị trí Y = 5 và Y = -5.

Đồ thị 3: Tiêu đề "Electromagnetic Torque". Trục Y: -5 đến 15. Gồm các đường Torque và đường đứt nét vàng biểu diễn Load Torque tại Y = 5.

Thanh điều khiển dưới đồ thị (QHBoxLayout):

QSlider nằm ngang (Qt.Horizontal) đại diện cho Time Window (giá trị hiển thị 5.0 s).

QLabel hiển thị dòng Tip: "Tip: SFMPC shows smaller overshoot, faster settling, and lower steady-state error." có icon bóng đèn.

D. Cột Phải - Thông Số Chi Tiết (QVBoxLayout)
Performance Metrics (QGroupBox): - Sử dụng QTableWidget gồm 4 cột: Metric, PID, SFMPC, Improvement.

Ẩn tiêu đề hàng (verticalHeader().setVisible(False)). Các ô số của cột PID có màu chữ cam, SFMPC màu chữ cyan, Improvement màu chữ xanh lá. Hoàn thiện với 6 hàng dữ liệu tương ứng.

System Status (QGroupBox):

Sử dụng QFormLayout hiển thị danh sách dạng Key-Value tĩnh.

Key: Chữ màu xám. Value: Chữ màu trắng (Riêng các giá trị trạng thái như RUNNING, RealTimeWorker_Active, ON thì có màu chữ xanh lá cây).

E. Footer Phụ (Góc dưới cùng bên phải)
Sử dụng một QHBoxLayout nhỏ chứa nút chuyển đổi Theme (Dark/Light switch icon), một nút tròn Play, và phần chỉnh cỡ chữ (Font Size kèm nút trừ -, nhãn 100%, và nút cộng +).

3. ĐỊNH PHONG CÁCH GIAO DIỆN (QSS - QT STYLE SHEETS)
Hãy tạo một chuỗi QSS tổng thể áp dụng cho ứng dụng để đạt được giao diện Deep Navy:

Mắt lưới đồ thị, nền cửa sổ: #060B19 (Dark Navy rất đậm).

Nền các QGroupBox, Card, Thẻ chỉ số: #0D162D.

Viền các Card/GroupBox: 1px solid #1E2E5D.

Màu chữ mặc định: #E2E8F0 (Trắng ngà). Title chính: #FFFFFF.

Màu sắc nhấn (Accent colors):

Cam (PID): #E65F2B

Xanh Cyan (SFMPC): #48CAE4

Xanh lá (Success/Improvement): #2ECC71

Định nghĩa bo góc border-radius: 4px hoặc 6px cho các nút bấm và ô nhập liệu để tạo cảm giác hiện đại.