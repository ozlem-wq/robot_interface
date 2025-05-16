import rospy
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt6.QtGui import QFont
from std_msgs.msg import Float32
import pyqtgraph as pg
from collections import deque


class PerformancePage(QWidget):
    def __init__(self):
        super().__init__()

        self.init_ui()
        self.init_ros()

        self.data_window = 100  # kaç veri gösterilecek (sürüklenen pencere)
        self.speed_data = deque(maxlen=self.data_window)
        self.battery_data = deque(maxlen=self.data_window)
        self.load_data = deque(maxlen=self.data_window)
        self.x = deque(maxlen=self.data_window)
        self.counter = 0

        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.update_graphs)
        self.timer.start(500)  # 0.5 saniyede bir çizim

    def init_ui(self):
        layout = QVBoxLayout()

        title = QLabel("📈 Anlık Performans Takibi")
        title.setFont(QFont("Arial", 18, QFont.Weight.Bold))
        title.setStyleSheet("color: white;")
        layout.addWidget(title)

        self.plot_widget = pg.PlotWidget(title="Speed / Battery / Load")
        self.plot_widget.setBackground('#111')
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.addLegend()

        self.speed_curve = self.plot_widget.plot(pen='y', name='Hız (RPM)')
        self.battery_curve = self.plot_widget.plot(pen='g', name='Pil (%)')
        self.load_curve = self.plot_widget.plot(pen='r', name='Yük (%)')

        layout.addWidget(self.plot_widget)
        self.setLayout(layout)

    def init_ros(self):
        rospy.Subscriber("/robot_speed", Float32, self.cb_speed)
        rospy.Subscriber("/battery_level", Float32, self.cb_battery)
        rospy.Subscriber("/load_level", Float32, self.cb_load)

    def cb_speed(self, msg):
        self.speed_data.append(msg.data)
        self.x.append(self.counter)
        self.counter += 1

    def cb_battery(self, msg):
        self.battery_data.append(msg.data)

    def cb_load(self, msg):
        self.load_data.append(msg.data)

    def update_graphs(self):
        self.speed_curve.setData(self.x, list(self.speed_data))
        self.battery_curve.setData(self.x, list(self.battery_data))
        self.load_curve.setData(self.x, list(self.load_data))

