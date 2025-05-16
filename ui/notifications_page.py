import rospy
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QListWidget, QMessageBox
from PyQt6.QtGui import QFont
from std_msgs.msg import String
from PyQt6.QtCore import Qt  # Qt'yi ekleyin


class NotificationsPage(QWidget):
    def __init__(self):
        super().__init__()
        self.setup_ui()

        self.notifications = []
        rospy.Subscriber("/robot_notifications", String, self.handle_notification)

    def setup_ui(self):
        layout = QVBoxLayout()

        title = QLabel("🔔 Bildirim Paneli")
        title.setFont(QFont("Arial", 24, QFont.Weight.Bold))
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet("color: white;")
        layout.addWidget(title)

        # Bildirimler listesi
        self.list_widget = QListWidget()
        self.list_widget.setStyleSheet("""
            background-color: #2E2E2E;
            color: white;
            border-radius: 10px;
            padding: 10px;
        """)
        layout.addWidget(self.list_widget)

        # Temizle butonu
        self.btn_clear = QPushButton("🧹 Bildirimleri Temizle")
        self.btn_clear.setStyleSheet("""
            background-color: #4CAF50;
            color: white;
            font-size: 16px;
            border-radius: 10px;
            padding: 10px;
        """)
        self.btn_clear.setStyleSheet("""
            QPushButton:hover {
                background-color: #45a049;
            }
        """)
        self.btn_clear.clicked.connect(self.clear_notifications)
        layout.addWidget(self.btn_clear)

        self.setLayout(layout)

    def handle_notification(self, msg):
        """Yeni bir bildirim geldiğinde listeye ekler"""
        self.notifications.append(msg.data)
        self.list_widget.addItem(msg.data)
        # Bildirim geldiğinde kullanıcıyı bilgilendir
        QMessageBox.information(self, "Yeni Bildirim", msg.data)

    def clear_notifications(self):
        """Bildirimleri temizle"""
        confirm = QMessageBox.question(
            self,
            "Bildirimleri Temizle",
            "Tüm bildirimler silinsin mi?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        if confirm == QMessageBox.StandardButton.Yes:
            self.notifications.clear()
            self.list_widget.clear()
            QMessageBox.information(self, "Temizlendi", "Tüm bildirimler temizlendi.")

