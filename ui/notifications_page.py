import rospy
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QListWidget, QPushButton, QMessageBox
from PyQt6.QtGui import QFont
from PyQt6.QtCore import Qt
from std_msgs.msg import String
import random

class NotificationsPage(QWidget):
    def __init__(self):
        super().__init__()
        self.setObjectName("notificationsPage")
        self.setup_ui()
        rospy.Subscriber("/robot_notifications", String, self.add_notification)

        # Dummy timer for test notification (remove in real robot)
        self.notification_timer()

    def setup_ui(self):
        layout = QVBoxLayout(self)

        title = QLabel("🔔 Bildirim Paneli")
        title.setFont(QFont("Arial", 18, QFont.Weight.Bold))
        title.setStyleSheet("color: white;")
        layout.addWidget(title)

        self.list_widget = QListWidget()
        layout.addWidget(self.list_widget)

        self.btn_clear = QPushButton("🧹 Tüm Bildirimleri Temizle")
        self.btn_clear.clicked.connect(self.clear_notifications)
        layout.addWidget(self.btn_clear)

    def add_notification(self, msg):
        self.list_widget.addItem(msg.data)

    def clear_notifications(self):
        if self.list_widget.count() > 0:
            confirm = QMessageBox.question(self, "Emin misiniz?", "Tüm bildirimleri silmek istiyor musunuz?",
                                           QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
            if confirm == QMessageBox.StandardButton.Yes:
                self.list_widget.clear()

    def notification_timer(self):
        from PyQt6.QtCore import QTimer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.generate_random_notification)
        self.timer.start(10000)  # 10 saniyede bir rastgele bildirim

    def generate_random_notification(self):
        warnings = [
            "🔋 Pil %20'nin altına düştü!",
            "📦 Yük kapasitesi aşıldı!",
            "🚧 Robot bir engele takıldı!",
            "📡 Bağlantı zayıf!",
            "🔄 Harita güncelleniyor..."
        ]
        warning = random.choice(warnings)
        self.add_notification(String(data=warning))

