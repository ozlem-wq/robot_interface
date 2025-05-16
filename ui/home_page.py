import os
import rospy
from PyQt6.QtWidgets import QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QDial, QProgressBar
from PyQt6.QtGui import QPixmap, QFont, QMovie
from PyQt6.QtCore import QTimer, Qt
from std_msgs.msg import Float32
from PyQt6.QtMultimediaWidgets import QVideoWidget
from PyQt6.QtMultimedia import QMediaPlayer, QAudioOutput, QMediaContent
from PyQt6.QtCore import QUrl
import datetime

class HomePage(QWidget):
    def __init__(self):
        super().__init__()
        self.setObjectName("HomePage")
        self.init_ui()
        self.start_time = None
        self.setup_ros_subscribers()
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_work_time)
        self.update_timer.start(1000)

    def init_ui(self):
        self.setStyleSheet("#HomePage { border-image: url(../images/background_home.png) 0 0 0 0 stretch stretch; }")

        layout = QVBoxLayout(self)

        # Üst menü butonları zaten main_window'da

        # Ana içerik düzeni
        content_layout = QHBoxLayout()

        # Sol kısım (hız, manuel kontrol, süre)
        left_panel = QVBoxLayout()

        self.label_speed_title = QLabel("🚀 Hız:")
        self.label_speed_title.setStyleSheet("color: white;")
        left_panel.addWidget(self.label_speed_title)

        self.dial_speed = QDial()
        self.dial_speed.setMinimum(0)
        self.dial_speed.setMaximum(120)
        self.dial_speed.setNotchesVisible(True)
        left_panel.addWidget(self.dial_speed)

        self.label_speed_value = QLabel("Hız: 0.0 RPM")
        self.label_speed_value.setStyleSheet("color: white;")
        left_panel.addWidget(self.label_speed_value)

        self.label_manual = QLabel("🕹 Manuel Kontrol")
        self.label_manual.setStyleSheet("color: orange;")
        left_panel.addWidget(self.label_manual)

        btn_row1 = QHBoxLayout()
        self.btn_forward = QPushButton("↑")
        btn_row1.addStretch()
        btn_row1.addWidget(self.btn_forward)
        btn_row1.addStretch()
        left_panel.addLayout(btn_row1)

        btn_row2 = QHBoxLayout()
        self.btn_left = QPushButton("←")
        self.btn_back = QPushButton("↓")
        self.btn_right = QPushButton("→")
        btn_row2.addWidget(self.btn_left)
        btn_row2.addWidget(self.btn_back)
        btn_row2.addWidget(self.btn_right)
        left_panel.addLayout(btn_row2)

        self.label_work = QLabel("⏱ Çalışma Süresi")
        self.label_work.setStyleSheet("color: white;")
        left_panel.addWidget(self.label_work)

        self.label_duration = QLabel("⏱ Süre: 00:00:00")
        self.label_duration.setStyleSheet("color: white;")
        left_panel.addWidget(self.label_duration)

        start_stop_layout = QHBoxLayout()
        self.btn_start = QPushButton("▶ Başlat")
        self.btn_stop = QPushButton("■ Durdur")
        start_stop_layout.addWidget(self.btn_start)
        start_stop_layout.addWidget(self.btn_stop)
        left_panel.addLayout(start_stop_layout)

        content_layout.addLayout(left_panel)

        # Sağ panel (video + göstergeler)
        right_panel = QVBoxLayout()

        self.label_video_title = QLabel("📷 Canlı İzleme")
        self.label_video_title.setStyleSheet("color: white;")
        right_panel.addWidget(self.label_video_title)

        # Canlı video QLabel (yuvarlak gibi göstermek için özel stil ile)
        self.video_label = QLabel()
        self.video_label.setFixedSize(300, 300)
        self.video_label.setStyleSheet("border-radius: 150px; border: 3px solid #00FFFF;")
        right_panel.addWidget(self.video_label, alignment=Qt.AlignmentFlag.AlignCenter)

        # Pil ve Yük Göstergesi
        self.label_status = QLabel("🔋 Pil ve Yük Durumu")
        self.label_status.setStyleSheet("color: white;")
        right_panel.addWidget(self.label_status)

        self.progress_battery = QProgressBar()
        self.progress_battery.setMaximum(100)
        self.progress_battery.setValue(0)
        self.progress_battery.setFormat("🔋 Pil: %p%%")
        right_panel.addWidget(self.progress_battery)

        self.progress_load = QProgressBar()
        self.progress_load.setMaximum(100)
        self.progress_load.setValue(0)
        self.progress_load.setFormat("📦 Yük (kg): %p%%")
        right_panel.addWidget(self.progress_load)

        content_layout.addLayout(right_panel)
        layout.addLayout(content_layout)

        self.init_video()

    def setup_ros_subscribers(self):
        rospy.Subscriber("/robot_speed", Float32, self.update_speed)
        rospy.Subscriber("/battery_level", Float32, self.update_battery)
        rospy.Subscriber("/load_level", Float32, self.update_load)

    def update_speed(self, msg):
        speed = msg.data
        self.dial_speed.setValue(int(speed * 100))  # RPM gösterim
        self.label_speed_value.setText(f"Hız: {speed:.1f} RPM")

    def update_battery(self, msg):
        self.progress_battery.setValue(int(msg.data))

    def update_load(self, msg):
        self.progress_load.setValue(int(msg.data))

    def update_work_time(self):
        if not self.start_time:
            self.start_time = datetime.datetime.now()
        elapsed = datetime.datetime.now() - self.start_time
        self.label_duration.setText("⏱ Süre: " + str(elapsed).split('.')[0])

    def init_video(self):
        import cv2
        self.cap = cv2.VideoCapture("../images/test_video.mp4")
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_video_frame)
        self.timer.start(33)

    def update_video_frame(self):
        import cv2
        from PyQt6.QtGui import QImage, QPixmap
        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                height, width, channels = frame.shape
                bytes_per_line = channels * width
                q_image = QImage(frame.data, width, height, bytes_per_line, QImage.Format.Format_RGB888)
                pixmap = QPixmap.fromImage(q_image)
                self.video_label.setPixmap(pixmap.scaled(self.video_label.size(), Qt.AspectRatioMode.KeepAspectRatio))
            else:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    def closeEvent(self, event):
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()
        event.accept()









