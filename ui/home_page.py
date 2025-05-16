import os
import cv2
import rospy
from PyQt6.QtWidgets import (
    QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout,
    QProgressBar, QDial, QSpacerItem, QSizePolicy
)
from PyQt6.QtGui import QPixmap, QImage, QFont
from PyQt6.QtCore import QTimer, Qt, QTime
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class HomePage(QWidget):
    def __init__(self):
        super().__init__()
        self.setObjectName("homePage")

        # Arka plan görseli tam ekran uygulanıyor
        bg_path = os.path.abspath("images/background_home.png").replace("\\", "/")
        self.setStyleSheet(f"""
            QWidget#homePage {{
                background-image: url("file:///{bg_path}");
                background-repeat: no-repeat;
                background-position: center;
                background-size: cover;
            }}
        """)

        self.bridge = None
        self.init_ros()
        self.setup_ui()

        self.start_time = QTime(0, 0, 0)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_video_frame)
        self.timer.start(30)
        self.video_capture = cv2.VideoCapture("images/test_video.mp4")

    def init_ros(self):
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/robot_speed", Float32, self.update_speed)
        rospy.Subscriber("/battery_level", Float32, self.update_battery)
        rospy.Subscriber("/load_level", Float32, self.update_load)

    def setup_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(30, 30, 30, 30)

        # Üst kısım: başlıklar ve canlı izleme
        top_layout = QHBoxLayout()

        # Sol üst: hız göstergesi
        speed_layout = QVBoxLayout()
        speed_label = QLabel("⚙ Hız:")
        speed_label.setStyleSheet("color: white; font-size: 14px;")
        self.label_speed = speed_label

        self.dial = QDial()
        self.dial.setNotchesVisible(True)
        self.dial.setMinimum(0)
        self.dial.setMaximum(100)
        self.dial.setFixedSize(100, 100)
        self.dial.setStyleSheet("QDial { background-color: lightblue; border-radius: 50px; }")

        speed_layout.addWidget(speed_label, alignment=Qt.AlignmentFlag.AlignCenter)
        speed_layout.addWidget(self.dial, alignment=Qt.AlignmentFlag.AlignCenter)
        top_layout.addLayout(speed_layout)

        # Orta üst: canlı izleme başlık + daire
        center_layout = QVBoxLayout()
        self.lbl_video_title = QLabel("📹 Canlı İzleme")
        self.lbl_video_title.setStyleSheet("color: white; font-weight: bold;")
        center_layout.addWidget(self.lbl_video_title, alignment=Qt.AlignmentFlag.AlignCenter)

        self.camera_label = QLabel()
        self.camera_label.setFixedSize(300, 300)
        self.camera_label.setStyleSheet("""
            border-radius: 150px;
            border: 4px solid cyan;
        """)
        center_layout.addWidget(self.camera_label, alignment=Qt.AlignmentFlag.AlignCenter)
        top_layout.addLayout(center_layout)

        # Sağ üst: süre ve göstergeler
        info_layout = QVBoxLayout()
        self.lbl_time = QLabel("⏱ Çalışma Süresi\nSüre: 00:00:00")
        self.lbl_time.setStyleSheet("color: white;")
        info_layout.addWidget(self.lbl_time, alignment=Qt.AlignmentFlag.AlignLeft)

        lbl_pil = QLabel("🔋 Pil")
        lbl_pil.setStyleSheet("color: white; font-weight: bold;")
        self.battery_bar = QProgressBar()
        self.battery_bar.setFormat("Pil: %p%%")
        self.battery_bar.setStyleSheet("QProgressBar {color: white;}")
        self.battery_bar.setFixedWidth(200)

        lbl_yuk = QLabel("📦 Yük (kg)")
        lbl_yuk.setStyleSheet("color: white; font-weight: bold;")
        self.load_bar = QProgressBar()
        self.load_bar.setFormat("Yük: %p%%")
        self.load_bar.setStyleSheet("QProgressBar {color: white;}")
        self.load_bar.setFixedWidth(200)

        info_layout.addWidget(lbl_pil)
        info_layout.addWidget(self.battery_bar)
        info_layout.addWidget(lbl_yuk)
        info_layout.addWidget(self.load_bar)

        top_layout.addLayout(info_layout)
        main_layout.addLayout(top_layout)

        # Orta: Manuel kontrol + başlat/durdur ortalanmış
        middle_layout = QVBoxLayout()
        middle_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)

        lbl_manual = QLabel("🕹 Manuel Kontrol")
        lbl_manual.setStyleSheet("color: orange; font-weight: bold;")
        middle_layout.addWidget(lbl_manual, alignment=Qt.AlignmentFlag.AlignCenter)

        self.btn_forward = QPushButton("↑")
        self.btn_back = QPushButton("↓")
        self.btn_left = QPushButton("←")
        self.btn_right = QPushButton("→")
        for btn in [self.btn_forward, self.btn_back, self.btn_left, self.btn_right]:
            btn.setFixedSize(50, 40)
            btn.clicked.connect(self.manual_control)

        arrow_grid = QVBoxLayout()
        arrow_grid.addWidget(self.btn_forward, alignment=Qt.AlignmentFlag.AlignCenter)
        mid_row = QHBoxLayout()
        mid_row.addWidget(self.btn_left)
        mid_row.addWidget(self.btn_right)
        arrow_grid.addLayout(mid_row)
        arrow_grid.addWidget(self.btn_back, alignment=Qt.AlignmentFlag.AlignCenter)
        middle_layout.addLayout(arrow_grid)

        # Başlat/durdur ortada
        control_buttons = QHBoxLayout()
        control_buttons.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.btn_start = QPushButton("▶ Başlat")
        self.btn_stop = QPushButton("■ Durdur")
        self.btn_start.setFixedWidth(120)
        self.btn_stop.setFixedWidth(120)
        self.btn_start.clicked.connect(self.start_timer)
        self.btn_stop.clicked.connect(self.stop_timer)
        control_buttons.addWidget(self.btn_start)
        control_buttons.addWidget(self.btn_stop)
        middle_layout.addLayout(control_buttons)

        main_layout.addLayout(middle_layout)

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
            frame = cv2.resize(frame, (300, 300))
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            q_img = QImage(frame.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
            self.camera_label.setPixmap(QPixmap.fromImage(q_img))

    def update_speed(self, msg):
        self.label_speed.setText(f"Hız: {msg.data:.1f} RPM")

    def update_battery(self, msg):
        self.battery_bar.setValue(int(msg.data))

    def update_load(self, msg):
        self.load_bar.setValue(int(msg.data))












