import os
import cv2
import rospy
import random
from PyQt6.QtWidgets import (
    QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QProgressBar, QDial,
    QSizePolicy, QSpacerItem, QGridLayout
)
from PyQt6.QtGui import QPixmap, QImage, QFont, QColor, QPainter
from PyQt6.QtCore import QTimer, Qt, QTime
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class HomePage(QWidget):
    def __init__(self):
        super().__init__()
        self.setObjectName("homePage")
        self.bg_pixmap = QPixmap("images/background_home.png")
        self.setAutoFillBackground(True)

        self.bridge = None
        self.init_ros()
        self.setup_ui()
        self.start_time = QTime(0, 0, 0)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_video_frame)
        self.timer.start(33)
        self.video_capture = cv2.VideoCapture("images/test_video.mp4")

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.drawPixmap(self.rect(), self.bg_pixmap)

    def init_ros(self):
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/robot_speed", Float32, self.update_speed)
        rospy.Subscriber("/battery_level", Float32, self.update_battery)
        rospy.Subscriber("/load_level", Float32, self.update_load)

    def setup_ui(self):
        ana_layout = QGridLayout(self)
        ana_layout.setContentsMargins(40, 80, 40, 40)
        ana_layout.setHorizontalSpacing(8)
        ana_layout.setVerticalSpacing(5)

        # 1. Sol üst: Hız göstergesi ve başlığı
        speed_vbox = QVBoxLayout()
        lbl_speed_title = QLabel("Hız: 0.0 RPM")
        lbl_speed_title.setFont(QFont("Segoe UI", 15, QFont.Weight.Bold))
        lbl_speed_title.setStyleSheet("color: #e0f6ff; margin-bottom: -4px;")
        lbl_speed_title.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        self.label_speed = lbl_speed_title
        speed_vbox.addWidget(lbl_speed_title, alignment=Qt.AlignmentFlag.AlignHCenter)
        self.dial = QDial()
        self.dial.setNotchesVisible(True)
        self.dial.setMinimum(0)
        self.dial.setMaximum(120)
        self.dial.setValue(0)
        self.dial.setFixedSize(95, 95)
        self.dial.setStyleSheet("QDial { background-color: #7ad2ff; border-radius: 47px; }")
        speed_vbox.addWidget(self.dial, alignment=Qt.AlignmentFlag.AlignHCenter)
        ana_layout.addLayout(speed_vbox, 0, 0, alignment=Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)

        # 2. Sol alt: Çalışma süresi - Modern mavi daire içinde, kırmızı yazılı
        sure_circle = QWidget()
        sure_circle.setFixedSize(135, 135)
        sure_circle.setStyleSheet(
            "background: qradialgradient(cx:0.5,cy:0.5,radius:0.9,fx:0.5,fy:0.5,stop:0 #02182B, stop:1 #12b5ff);"
            "border: 5px solid #12b5ff; border-radius:67px;"
        )
        sure_circle_layout = QVBoxLayout(sure_circle)
        sure_circle_layout.setContentsMargins(6, 14, 6, 6)
        sure_circle_layout.setSpacing(1)

        lbl_sure_title = QLabel("⏱ Çalışma Süresi")
        lbl_sure_title.setFont(QFont("Segoe UI", 11, QFont.Weight.Bold))
        lbl_sure_title.setStyleSheet("color: #e0f6ff;")
        lbl_sure_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        sure_circle_layout.addWidget(lbl_sure_title)

        self.lbl_time = QLabel("00:00:00")
        self.lbl_time.setFont(QFont("Consolas", 17, QFont.Weight.Bold))
        self.lbl_time.setStyleSheet("color: #ff4545; background:transparent;")
        self.lbl_time.setAlignment(Qt.AlignmentFlag.AlignCenter)
        sure_circle_layout.addWidget(self.lbl_time)

        ana_layout.addWidget(sure_circle, 2, 0, alignment=Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignBottom)

        # 3. Orta: Canlı İzleme dairesel video
        video_vbox = QVBoxLayout()
        self.lbl_video_title = QLabel("📹 Canlı İzleme")
        self.lbl_video_title.setFont(QFont("Segoe UI", 15, QFont.Weight.Bold))
        self.lbl_video_title.setStyleSheet("color: #e0f6ff; margin-bottom: 4px;")
        self.lbl_video_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        video_vbox.addWidget(self.lbl_video_title, alignment=Qt.AlignmentFlag.AlignCenter)
        self.camera_label = QLabel()
        self.camera_label.setFixedSize(390, 390)
        self.camera_label.setStyleSheet("border-radius:195px; border:6px solid #1cf0ff; background:rgba(0,0,0,0.1);")
        video_vbox.addWidget(self.camera_label, alignment=Qt.AlignmentFlag.AlignCenter)

        # PIL/YÜK - Video altı
        self.battery_title = QLabel("🔋 Pil:")
        self.battery_title.setStyleSheet("color: #1cf0ff; font-weight:bold; font-size: 14px; margin-top: 20px;")
        self.battery_bar = QProgressBar()
        self.battery_bar.setFixedWidth(270)
        self.battery_bar.setStyleSheet("QProgressBar {color: #e0f6ff; background-color:#102e3a;} QProgressBar::chunk {background-color:#28e3cb;}")
        self.load_title = QLabel("📦 Yük:")
        self.load_title.setStyleSheet("color: #1cf0ff; font-weight:bold; font-size: 14px; margin-top: 5px;")
        self.load_bar = QProgressBar()
        self.load_bar.setFixedWidth(270)
        self.load_bar.setStyleSheet("QProgressBar {color: #e0f6ff; background-color:#102e3a;} QProgressBar::chunk {background-color:#45a8f3;}")

        pil_yuk_vbox = QVBoxLayout()
        pil_yuk_vbox.setAlignment(Qt.AlignmentFlag.AlignTop)
        pil_yuk_vbox.addWidget(self.battery_title)
        pil_yuk_vbox.addWidget(self.battery_bar)
        pil_yuk_vbox.addWidget(self.load_title)
        pil_yuk_vbox.addWidget(self.load_bar)
        video_vbox.addLayout(pil_yuk_vbox)

        ana_layout.addLayout(video_vbox, 0, 1, 2, 1, alignment=Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignHCenter)

        # BAŞLAT/DURDUR - CANLI İZLEME SAĞI
        btns_hbox = QHBoxLayout()
        self.btn_start = QPushButton("▶ Başlat")
        self.btn_stop = QPushButton("■ Durdur")
        self.btn_start.setFixedSize(135, 38)
        self.btn_stop.setFixedSize(135, 38)
        self.btn_start.setFont(QFont("Segoe UI", 14, QFont.Weight.Bold))
        self.btn_stop.setFont(QFont("Segoe UI", 14, QFont.Weight.Bold))
        self.btn_start.setStyleSheet("background-color:#10c77b;color:white;border-radius:8px;")
        self.btn_stop.setStyleSheet("background-color:#e5505c;color:white;border-radius:8px;")
        btns_hbox.addWidget(self.btn_start)
        btns_hbox.addSpacing(36)
        btns_hbox.addWidget(self.btn_stop)
        ana_layout.addLayout(btns_hbox, 0, 2, alignment=Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        self.btn_start.clicked.connect(self.start_timer)
        self.btn_stop.clicked.connect(self.stop_timer)

        # MANUEL KONTROL - CANLI İZLEME SAĞ ALT
        manuel_vbox = QVBoxLayout()
        manuel_vbox.setSpacing(7)
        lbl_manual = QLabel("🕹 Manuel Kontrol")
        lbl_manual.setFont(QFont("Segoe UI", 14, QFont.Weight.Bold))
        lbl_manual.setStyleSheet("color: orange; margin-bottom: 5px;")
        lbl_manual.setAlignment(Qt.AlignmentFlag.AlignCenter)
        manuel_vbox.addWidget(lbl_manual, alignment=Qt.AlignmentFlag.AlignCenter)
        btn_size = (48, 34)
        self.btn_forward = QPushButton("↑")
        self.btn_left   = QPushButton("←")
        self.btn_right  = QPushButton("→")
        self.btn_back   = QPushButton("↓")
        for btn in [self.btn_forward, self.btn_left, self.btn_right, self.btn_back]:
            btn.setFixedSize(*btn_size)
            btn.setFont(QFont("Segoe UI", 15, QFont.Weight.Bold))
            btn.setStyleSheet(
                "QPushButton {background-color:#262f39;color:#baf1ff;border-radius:10px;border:2px solid #3ca4dc;}"
                "QPushButton:hover {background-color:#3ca4dc; color:#222;}"
            )
            btn.clicked.connect(self.manual_control)
        arrow_grid = QGridLayout()
        arrow_grid.setHorizontalSpacing(8)
        arrow_grid.setVerticalSpacing(5)
        arrow_grid.addWidget(self.btn_forward, 0, 1)
        arrow_grid.addWidget(self.btn_left,   1, 0)
        arrow_grid.addWidget(self.btn_right,  1, 2)
        arrow_grid.addWidget(self.btn_back,   2, 1)
        manuel_vbox.addLayout(arrow_grid)
        ana_layout.addLayout(manuel_vbox, 2, 2, alignment=Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignBottom)

    # ---- ROS ve UI Fonksiyonları ----
    def manual_control(self):
        sender = self.sender()
        twist = Twist()
        if sender == self.btn_forward:
            twist.linear.x = 0.5
        elif sender == self.btn_back:
            twist.linear.x = -0.5
        elif sender == self.btn_left:
            twist.angular.z = 0.5
        elif sender == self.btn_right:
            twist.angular.z = -0.5
        self.cmd_pub.publish(twist)

    def start_timer(self):
        self.start_time = QTime.currentTime()
        self.update_elapsed()
        self.elapsed_timer = QTimer()
        self.elapsed_timer.timeout.connect(self.update_elapsed)
        self.elapsed_timer.start(1000)

    def stop_timer(self):
        if hasattr(self, 'elapsed_timer'):
            self.elapsed_timer.stop()

    def update_elapsed(self):
        elapsed = QTime(0, 0).secsTo(QTime.currentTime()) - QTime(0, 0).secsTo(self.start_time)
        self.lbl_time.setText(f"{QTime(0, 0).addSecs(elapsed).toString('hh:mm:ss')}")

    def update_video_frame(self):
        if self.video_capture.isOpened():
            ret, frame = self.video_capture.read()
            if not ret:
                self.video_capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                return
            frame = cv2.resize(frame, (390, 390))
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = frame.shape
            mask = self.create_circle_mask(h, w)
            frame[mask == 0] = (0, 0, 0)
            q_img = QImage(frame.data, w, h, ch * w, QImage.Format.Format_RGB888)
            self.camera_label.setPixmap(QPixmap.fromImage(q_img))

    def create_circle_mask(self, h, w):
        import numpy as np
        Y, X = np.ogrid[:h, :w]
        center = (h // 2, w // 2)
        radius = min(center[0], center[1], w//2)-2
        mask = (X - center[1])**2 + (Y - center[0])**2 <= radius**2
        return mask.astype('uint8') * 255

    def update_speed(self, msg):
        self.label_speed.setText(f"Hız: {msg.data:.1f} RPM")
        self.dial.setValue(int(msg.data))

    def update_battery(self, msg):
        self.battery_bar.setValue(int(msg.data))

    def update_load(self, msg):
        self.load_bar.setValue(int(msg.data))

