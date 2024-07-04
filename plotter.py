import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

# Cấu hình cổng COM và tốc độ baud rate
port = 'COM15'  # Thay bằng cổng COM của bạn
baud_rate = 9600

# Mở kết nối serial
ser = serial.Serial(port, baud_rate)

# Danh sách để lưu trữ dữ liệu và thời gian
data = []
timestamps = []

# Thời gian bắt đầu
start_time = time.time()

# Hàm cập nhật dữ liệu và vẽ biểu đồ
def update(frame):
    global data, timestamps
    if ser.in_waiting >= 4:  # Kiểm tra có ít nhất 2 byte trong buffer
        bytes_to_read = ser.read(4)  # Đọc 2 byte
        value = int.from_bytes(bytes_to_read, byteorder='big')  # Chuyển đổi 2 byte thành số nguyên
        current_time = time.time()
        elapsed_time_ms = (current_time - start_time) * 1000  # Tính thời gian đã trôi qua bằng milli giây
        data.append(value)  # Thêm dữ liệu vào danh sách
        timestamps.append(elapsed_time_ms)  # Thêm thời gian vào danh sách
        if len(data) > 100:  # Giữ dữ liệu trong phạm vi 100 phần tử
            data.pop(0)
            timestamps.pop(0)
        plt.cla()  # Xóa biểu đồ hiện tại
        plt.plot(timestamps, data)  # Vẽ biểu đồ mới
        plt.xlabel('Time (ms)')
        plt.ylabel('Value')
        plt.title('Real-time UART Data Plot')
        plt.tight_layout()

# Tạo biểu đồ động
ani = FuncAnimation(plt.gcf(), update, interval=100)

# Hiển thị biểu đồ
plt.show()

# Đóng kết nối serial khi kết thúc (nếu cần)
# ser.close()  # Thực hiện điều này khi bạn muốn đóng kết nối serial
