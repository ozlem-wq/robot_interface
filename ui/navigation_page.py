import os
import rospy
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QPushButton, QComboBox, QFileDialog, QMessageBox
)
from PyQt6.QtGui import QPixmap, QPainter, QColor, QPen, QFont
from PyQt6.QtCore import Qt
from std_msgs.msg import String

class NavigationPage(QWidget):
    def __init__(self):
        super().__init__()

        self.image_dir = os.path.join(os.path.dirname(__file__), "../images")
        self.map_paths = self.load_maps()

        self.publisher_target = rospy.Publisher("/send_robot_to_goal", String, queue_size=10)
        self.publisher_reset = rospy.Publisher("/reset_robot_position", String, queue_size=10)

        self.setup_ui()

    def load_maps(self):
        maps = {}
        for fname in os.listdir(self.image_dir):
            if fname.endswith(".ppm"):
                label = os.path.splitext(fname)[0]
                maps[label] = os.path.join(self.image_dir, fname)
        return maps

    def setup_ui(self):
        layout = QVBoxLayout()

        title = QLabel("🗺 Navigasyon ve Harita İzleme")
        title.setFont(QFont("Arial", 18, QFont.Weight.Bold))
        title.setStyleSheet("color: white;")
        layout.addWidget(title)

        self.map_selector = QComboBox()
        self.map_selector.addItems(self.map_paths.keys())
        self.map_selector.currentTextChanged.connect(self.update_map)
        layout.addWidget(self.map_selector)

        self.map_display = QLabel("Harita gösterimi burada olacaktır.")
        self.map_display.setFixedSize(640, 480)
        self.map_display.setStyleSheet("border: 2px solid #444;")
        layout.addWidget(self.map_display)

        # Gönder ve sıfırla
        self.btn_send = QPushButton("📍 Hedefe Git")
        self.btn_send.clicked.connect(self.send_goal)
        self.btn_send.setStyleSheet("color: white; background-color: #2d6cdf;")
        layout.addWidget(self.btn_send)

        self.btn_reset = QPushButton("🔄 Konumu Sıfırla")
        self.btn_reset.clicked.connect(self.reset_position)
        self.btn_reset.setStyleSheet("color: white; background-color: #e06666;")
        layout.addWidget(self.btn_reset)

        # Harita ekle/sil
        self.btn_add = QPushButton("📁 Yeni Harita Ekle")
        self.btn_add.clicked.connect(self.add_map)
        layout.addWidget(self.btn_add)

        self.btn_delete = QPushButton("🗑 Seçili Haritayı Sil")
        self.btn_delete.clicked.connect(self.delete_map)
        layout.addWidget(self.btn_delete)

        self.setLayout(layout)

        if self.map_paths:
            self.update_map(next(iter(self.map_paths)))

    def update_map(self, map_name):
        map_path = self.map_paths.get(map_name)
        if map_path and os.path.exists(map_path):
            pixmap = QPixmap(map_path)
            painter = QPainter(pixmap)
            pen = QPen(QColor("red"))
            painter.setPen(pen)
            painter.setBrush(QColor("red"))
            painter.drawEllipse(100, 100, 10, 10)
            painter.end()
            self.map_display.setPixmap(pixmap.scaled(
                self.map_display.size(), Qt.AspectRatioMode.KeepAspectRatio))

    def send_goal(self):
        self.publisher_target.publish("goal_sent")
        rospy.loginfo("📍 Hedefe git komutu gönderildi.")

    def reset_position(self):
        self.publisher_reset.publish("reset_position")
        rospy.loginfo("🔄 Robot pozisyonu sıfırlandı.")

    def add_map(self):
        dialog = QFileDialog()
        file_path, _ = dialog.getOpenFileName(self, "Yeni Harita Seç", "", "PPM Files (*.ppm)")
        if file_path:
            fname = os.path.basename(file_path)
            dest_path = os.path.join(self.image_dir, fname)
            if not os.path.exists(dest_path):
                from shutil import copyfile
                copyfile(file_path, dest_path)
                label = os.path.splitext(fname)[0]
                self.map_paths[label] = dest_path
                self.map_selector.addItem(label)
                QMessageBox.information(self, "Harita Eklendi", f"{label} başarıyla eklendi.")
            else:
                QMessageBox.warning(self, "Mevcut", "Bu isimde bir harita zaten var.")

    def delete_map(self):
        selected_label = self.map_selector.currentText()
        map_path = self.map_paths.get(selected_label)
        if map_path and os.path.exists(map_path):
            confirm = QMessageBox.question(
                self, "Silme Onayı",
                f"{selected_label} haritasını silmek istediğinize emin misiniz?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
            )
            if confirm == QMessageBox.StandardButton.Yes:
                os.remove(map_path)
                del self.map_paths[selected_label]
                self.map_selector.removeItem(self.map_selector.currentIndex())
                self.map_display.setText("Harita silindi.")




