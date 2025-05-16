import os
import cv2
import rospy
import random
from PyQt6.QtWidgets import (
    QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QProgressBar, QDial, QSizePolicy, QSpacerItem
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
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(40, 70, 40, 40)
        main_layout.setSpacing(10)

        # ------ Sol panel: Hız göstergesi ------
        left_panel = QVBoxLayout()
        left_panel.setAlignment(Qt.AlignmentFlag.AlignTop)
        lbl_speed_title = QLabel("Hız: 0.0 RPM")
        lbl_speed_title.setFont(QFont("Segoe UI", 15, QFont.Weight.Bold))
        lbl_speed_title.setStyleSheet("color: #e0f6ff;")
        lbl_speed_title.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        self.label_speed = lbl_speed_title
        left_panel.addWidget(lbl_speed_title)
        self.dial = QDial()
        self.dial.setNotchesVisible(True)
        self.dial.setMinimum(0)
        self.dial.setMaximum(120)
        self.dial.setValue(0)
        self.dial.setFixedSize(110, 110)
        self.dial.setStyleSheet("QDial { background-color: #7ad2ff; border-radius: 55px; }")
        left_panel.addWidget(self.dial, alignment=Qt.AlignmentFlag.AlignHCenter)
        left_panel.addSpacing(20)

        main_layout.addLayout(left_panel, 1)

        # ------ Orta panel: Butonlar ve video ------
        center_vbox = QVBoxLayout()
        center_vbox.setAlignment(Qt.AlignmentFlag.AlignCenter)

        # Canlı İzleme başlık
        self.lbl_video_title = QLabel("📹 Canlı İzleme")
        self.lbl_video_title.setFont(QFont("Segoe UI", 14, QFont.Weight.Bold))
        self.lbl_video_title.setStyleSheet("color: #e0f6ff;")
        self.lbl_video_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        center_vbox.addWidget(self.lbl_video_title)

        # Canlı izleme video (tam mavi dairenin ortasına)
        self.camera_label = QLabel()
        self.camera_label.setFixedSize(350, 350)
        self.camera_label.setStyleSheet(
            "border-radius:185px; border:5px solid #1cf0ff; background:rgba(0,0,0,0.2);"
        )
        # Genişletmek ve tam ortaya almak için bir QHBoxLayout ile hizalama
        video_row = QHBoxLayout()
        video_row.addSpacerItem(QSpacerItem(580, 10, QSizePolicy.Policy.Expanding))
        video_row.addWidget(self.camera_label, alignment=Qt.AlignmentFlag.AlignCenter)
        video_row.addSpacerItem(QSpacerItem(10, 10, QSizePolicy.Policy.Expanding))
        center_vbox.addLayout(video_row)
        center_vbox.addSpacing(200)

        # --- Manuel kontrol ve başlat/durdur butonlarını ortada hizala ---
        button_block = QVBoxLayout()
        button_block.setAlignment(Qt.AlignmentFlag.AlignCenter)
        # Manuel Kontrol Başlık
        lbl_manual = QLabel("🕹 Manuel Kontrol")
        lbl_manual.setFont(QFont("Segoe UI", 13, QFont.Weight.Bold))
        lbl_manual.setStyleSheet("color: orange; margin-bottom: 5px;")
        button_block.addWidget(lbl_manual, alignment=Qt.AlignmentFlag.AlignHCenter)

        # Ok Butonları
        arrow_grid = QVBoxLayout()
        arrow_grid.setSpacing(2)
        btn_size = (44, 32)
        self.btn_forward = QPushButton("↑")
        self.btn_left   = QPushButton("←")
        self.btn_right  = QPushButton("→")
        self.btn_back   = QPushButton("↓")
        for btn in [self.btn_forward, self.btn_left, self.btn_right, self.btn_back]:
            btn.setFixedSize(*btn_size)
            btn.setFont(QFont("Segoe UI", 15, QFont.Weight.Bold))
            btn.setStyleSheet(
                "QPushButton {background-color:#262f39;color:#baf1ff;border-radius:9px;border:1.5px solid #3ca4dc;}"
                "QPushButton:hover {background-color:#3ca4dc; color:#222;}"
            )
            btn.clicked.connect(self.manual_control)

        arrow_grid.addWidget(self.btn_forward, alignment=Qt.AlignmentFlag.AlignCenter)
        mid_row = QHBoxLayout()
        mid_row.setSpacing(10)
        mid_row.addWidget(self.btn_left)
        mid_row.addWidget(self.btn_right)
        arrow_grid.addLayout(mid_row)
        arrow_grid.addWidget(self.btn_back, alignment=Qt.AlignmentFlag.AlignCenter)
        button_block.addLayout(arrow_grid)
        button_block.addSpacing(10)

        # Süre Takibi
        self.lbl_time = QLabel("⏱ Çalışma Süresi\nSüre: 00:00:00")
        self.lbl_time.setStyleSheet("color: #e0f6ff; font-size: 14px;")
        button_block.addWidget(self.lbl_time, alignment=Qt.AlignmentFlag.AlignHCenter)
        button_block.addSpacing(7)

        # Başlat / Durdur Butonları (yan yana)
        control_buttons = QHBoxLayout()
        self.btn_start = QPushButton("▶ Başlat")
        self.btn_stop = QPushButton("■ Durdur")
        for btn, col in zip([self.btn_start, self.btn_stop], ["#10c77b", "#e5505c"]):
            btn.setFixedSize(120, 36)
            btn.setFont(QFont("Segoe UI", 13, QFont.Weight.Bold))
            btn.setStyleSheet(
                f"QPushButton {{background-color:{col};color:white;border-radius:8px;}}"
                f"QPushButton:hover {{background-color:#31b0e7; color:#fff;}}"
            )
        self.btn_start.clicked.connect(self.start_timer)
        self.btn_stop.clicked.connect(self.stop_timer)
        control_buttons.addWidget(self.btn_start)
        control_buttons.addWidget(self.btn_stop)
        button_block.addLayout(control_buttons)

        center_vbox.addLayout(button_block)
        main_layout.addLayout(center_vbox, 3)

        # ------ Sağ panel: Pil/Yük ------
        right_panel = QVBoxLayout()
        right_panel.setAlignment(Qt.AlignmentFlag.AlignBottom | Qt.AlignmentFlag.AlignRight)
        right_panel.addSpacing(160)
        self.battery_bar = QProgressBar()
        self.battery_bar.setFormat("🔋 Pil: %p%%")
        self.battery_bar.setFixedWidth(280)
        self.battery_bar.setStyleSheet(
            "QProgressBar {color: #e0f6ff; font-weight:bold; background-color:#102e3a;}"
            "QProgressBar::chunk {background-color:#28e3cb;}"
        )
        right_panel.addWidget(self.battery_bar, alignment=Qt.AlignmentFlag.AlignCenter)
        self.load_bar = QProgressBar()
        self.load_bar.setFormat("📦 Yük (kg): %p%%")
        self.load_bar.setFixedWidth(280)
        self.load_bar.setStyleSheet(
            "QProgressBar {color: #e0f6ff; font-weight:bold; background-color:#102e3a;}"
            "QProgressBar::chunk {background-color:#45a8f3;}"
        )
        right_panel.addWidget(self.load_bar, alignment=Qt.AlignmentFlag.AlignCenter)
        main_layout.addLayout(right_panel, 1)

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
        self.lbl_time.setText(f"⏱ Çalışma Süresi\nSüre: {QTime(0, 0).addSecs(elapsed).toString('hh:mm:ss')}")

    def update_video_frame(self):
        if self.video_capture.isOpened():
            ret, frame = self.video_capture.read()
            if not ret:
                self.video_capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                return
            frame = cv2.resize(frame, (370, 370))
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














