import os
import rospy
import cv2
from PyQt6.QtWidgets import (
    QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout,
    QProgressBar, QDial, QSizePolicy
)
from PyQt6.QtCore import Qt, QTimer, QTime
from PyQt6.QtGui import QPixmap, QFont, QImage
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from ui.settings_page import SettingsPage


class HomePage(QWidget):
    def __init__(self):
        super().__init__()
        self.start_time = QTime.currentTime()
        self.elapsed_timer = QTimer()
        self.elapsed_timer.timeout.connect(self.update_runtime)
        self.elapsed_timer.start(1000)

        self.apply_stylesheet()
        self.setup_ui()
        self.setup_video_player()
        self.setup_subscribers()

    def apply_stylesheet(self):
        qss_path = os.path.join(os.path.dirname(__file__), "style.qss")
        if os.path.exists(qss_path):
            with open(qss_path, "r") as f:
                self.setStyleSheet(f.read())

    def setup_ui(self):
        self.setObjectName("homePage")
        self.setAutoFillBackground(True)

        self.background = QLabel(self)
        pixmap = QPixmap(os.path.join(os.path.dirname(__file__), "../images/background_home.png"))
        self.background.setPixmap(pixmap)
        self.background.setScaledContents(True)
        self.background.setGeometry(0, 0, 1280, 720)
        self.background.lower()

        self.label_speed_title = QLabel("⚙️ Hız:", self)
        self.label_speed_title.move(50, 40)
        self.label_speed_title.setStyleSheet("color: white; font-size: 14px;")

        self.label_speed = QLabel("Hız: 0.0 RPM", self)
        self.label_speed.move(50, 130)
        self.label_speed.setStyleSheet("color: white;")

        self.dial_speed = QDial(self)
        self.dial_speed.setGeometry(50, 70, 100, 100)
        self.dial_speed.setRange(0, 100)

        self.label_manual = QLabel("🕹 Manuel Kontrol", self)
        self.label_manual.move(50, 180)
        self.label_manual.setStyleSheet("color: orange; font-weight: bold;")

        self.btn_forward = QPushButton("↑", self)
        self.btn_backward = QPushButton("↓", self)
        self.btn_left = QPushButton("←", self)
        self.btn_right = QPushButton("→", self)

        self.btn_forward.setGeometry(140, 210, 50, 30)
        self.btn_backward.setGeometry(140, 250, 50, 30)
        self.btn_left.setGeometry(80, 250, 50, 30)
        self.btn_right.setGeometry(200, 250, 50, 30)

        self.label_runtime_title = QLabel("⏱ Çalışma Süresi", self)
        self.label_runtime_title.move(50, 300)
        self.label_runtime_title.setStyleSheet("color: white;")

        self.label_runtime = QLabel("Süre: 00:00:00", self)
        self.label_runtime.move(50, 330)
        self.label_runtime.setStyleSheet("color: white;")

        self.btn_start = QPushButton("▶ Başlat", self)
        self.btn_stop = QPushButton("■ Durdur", self)
        self.btn_start.setGeometry(50, 620, 180, 30)
        self.btn_stop.setGeometry(250, 620, 180, 30)

        self.label_video_title = QLabel("📹 Canlı İzleme", self)
        self.label_video_title.move(830, 130)
        self.label_video_title.setStyleSheet("color: white; font-size: 16px;")

        self.label_video = QLabel(self)
        self.label_video.setGeometry(780, 160, 260, 260)
        self.label_video.setStyleSheet("border-radius: 130px; background-color: black;")
        self.label_video.setScaledContents(True)

        self.label_battery = QLabel("🔋 Pil:", self)
        self.label_battery.move(780, 450)
        self.label_battery.setStyleSheet("color: white;")
        self.progress_battery = QProgressBar(self)
        self.progress_battery.setGeometry(830, 450, 300, 20)

        self.label_load = QLabel("📦 Yük (kg):", self)
        self.label_load.move(780, 480)
        self.label_load.setStyleSheet("color: white;")
        self.progress_load = QProgressBar(self)
        self.progress_load.setGeometry(870, 480, 260, 20)

        # Butonlara hareket bağlantısı
        self.btn_forward.clicked.connect(lambda: self.send_cmd_vel(0.5, 0.0))
        self.btn_backward.clicked.connect(lambda: self.send_cmd_vel(-0.5, 0.0))
        self.btn_left.clicked.connect(lambda: self.send_cmd_vel(0.0, 0.5))
        self.btn_right.clicked.connect(lambda: self.send_cmd_vel(0.0, -0.5))

    def setup_video_player(self):
        video_path = os.path.join(os.path.dirname(__file__), "../images/test_video.mp4")
        self.cap = cv2.VideoCapture(video_path)
        self.video_timer = QTimer(self)
        self.video_timer.timeout.connect(self.update_frame)
        self.video_timer.start(30)

    def update_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = cv2.resize(frame, (260, 260))
        image = QImage(frame.data, frame.shape[1], frame.shape[0], QImage.Format.Format_RGB888)
        pixmap = QPixmap.fromImage(image)
        self.label_video.setPixmap(pixmap)

    def update_runtime(self):
        elapsed = QTime.currentTime().secsTo(self.start_time)
        if elapsed < 0:
            elapsed = -elapsed
        hours = elapsed // 3600
        minutes = (elapsed % 3600) // 60
        seconds = elapsed % 60
        self.label_runtime.setText(f"Süre: {hours:02}:{minutes:02}:{seconds:02}")

    def setup_subscribers(self):
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/robot_speed", Float32, self.update_speed)
        rospy.Subscriber("/battery_level", Float32, self.update_battery)
        rospy.Subscriber("/load_level", Float32, self.update_load)

    def update_speed(self, msg):
        self.label_speed.setText(f"Hız: {msg.data:.1f} RPM")
        self.dial_speed.setValue(int(msg.data))

    def update_battery(self, msg):
        self.progress_battery.setValue(int(msg.data))

    def update_load(self, msg):
        self.progress_load.setValue(int(msg.data))

    def send_cmd_vel(self, lin, ang):
        twist = Twist()
        twist.linear.x = lin
        twist.angular.z = ang
        self.cmd_vel_pub.publish(twist)










