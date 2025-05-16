# performance_page.py

import rospy
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton
from PyQt6.QtGui import QFont
from std_msgs.msg import Float32
import pyqtgraph as pg

class PerformancePage(QWidget):
    def __init__(self):
        super().__init__()
        self.setup_ui()

        self.time_data = []
        self.speed_data = []
        self.battery_data = []
        self.load_data = []
        self.counter = 0

        rospy.Subscriber("/robot_speed", Float32, self.update_speed)
        rospy.Subscriber("/battery_level", Float32, self.update_battery)
        rospy.Subscriber("/load_level", Float32, self.update_load)

    def setup_ui(self):
        layout = QVBoxLayout()

        title = QLabel("📊 Performans Grafikleri")
        title.setFont(QFont("Arial", 18))
        layout.addWidget(title)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setTitle("Anlık Hız Değeri")
        self.plot_widget.setLabel("left", "Hız (m/s)")
        self.plot_widget.setLabel("bottom", "Zaman (s)")
        layout.addWidget(self.plot_widget)

        self.btn_export = QPushButton("📁 Veriyi CSV Olarak Kaydet")
        self.btn_export.clicked.connect(self.export_data_csv)
        layout.addWidget(self.btn_export)

        self.setLayout(layout)

    def update_speed(self, msg):
        self.counter += 1
        self.time_data.append(self.counter)
        self.speed_data.append(msg.data)
        self.update_graph()

    def update_battery(self, msg):
        self.battery_data.append(msg.data)

    def update_load(self, msg):
        self.load_data.append(msg.data)

    def update_graph(self):
        self.plot_widget.clear()
        self.plot_widget.plot(self.time_data, self.speed_data, pen='r')

    def export_data_csv(self):
        file_name = "performance_data.csv"
        try:
            with open(file_name, "w") as f:
                f.write("Zaman,Hiz,Pil,Yuk\n")
                for i in range(len(self.speed_data)):
                    hiz = self.speed_data[i]
                    pil = self.battery_data[i] if i < len(self.battery_data) else 0
                    yuk = self.load_data[i] if i < len(self.load_data) else 0
                    f.write(f"{i},{hiz},{pil},{yuk}\n")
            print(f"Veriler {file_name} dosyasına kaydedildi!")
        except Exception as e:
            print("CSV dışa aktarma hatası:", e)
