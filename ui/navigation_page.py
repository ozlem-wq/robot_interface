import os
import shutil
import rospy
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QPushButton, QComboBox,
    QFileDialog, QMessageBox
)
from PyQt6.QtGui import QPixmap, QFont
from PyQt6.QtCore import Qt
from std_msgs.msg import String

class NavigationPage(QWidget):
    def __init__(self):
        super().__init__()

        self.image_dir = os.path.expanduser("~/catkin_ws/src/robot_interface/images")
        self.map_paths = self.load_existing_maps()

        self.setup_ui()

        self.publisher_target = rospy.Publisher("/send_robot_to_goal", String, queue_size=10)
        self.publisher_reset = rospy.Publisher("/reset_robot_position", String, queue_size=10)

    def load_existing_maps(self):
        maps = {}
        for fname in os.listdir(self.image_dir):
            if fname.endswith((".ppm", ".png", ".jpg")):
                label = os.path.splitext(fname)[0].capitalize().replace("_", " ")
                maps[label] = os.path.join(self.image_dir, fname)
        return maps

    def setup_ui(self):
        layout = QVBoxLayout()
        layout.setSpacing(12)
        layout.setContentsMargins(40, 20, 40, 20)
        self.setStyleSheet("background-color: black; color: white; font-family: Arial;")

        title = QLabel("🗺 Navigasyon ve Harita İzleme")
        title.setFont(QFont("Arial", 20, QFont.Weight.Bold))
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)

        # Harita seçimi
        label_combo = QLabel("📍 Harita Seç:")
        label_combo.setFont(QFont("Arial", 12))
        layout.addWidget(label_combo)

        self.map_selector = QComboBox()
        self.map_selector.addItems(self.map_paths.keys())
        self.map_selector.currentTextChanged.connect(self.update_map_display)
        layout.addWidget(self.map_selector)

        # Harita gösterimi
        self.map_display = QLabel("Harita yüklenecek...")
        self.map_display.setFixedSize(640, 480)
        self.map_display.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.map_display.setStyleSheet("border: 2px solid white;")
        layout.addWidget(self.map_display)

        # Hedefe git butonu
        label_goal = QLabel("🚀 Robotu Gönder:")
        label_goal.setFont(QFont("Arial", 12))
        layout.addWidget(label_goal)

        btn_send_goal = QPushButton("📤 Hedefe Git")
        btn_send_goal.setStyleSheet("color: white; font-weight: bold;")
        btn_send_goal.clicked.connect(self.send_goal)
        layout.addWidget(btn_send_goal)

        # Konumu sıfırla
        label_reset = QLabel("🔄 Konumu Sıfırla:")
        label_reset.setFont(QFont("Arial", 12))
        layout.addWidget(label_reset)

        btn_reset = QPushButton("🔁 Sıfırla")
        btn_reset.setStyleSheet("color: white; font-weight: bold;")
        btn_reset.clicked.connect(self.reset_position)
        layout.addWidget(btn_reset)

        # Harita ekle
        label_add = QLabel("📁 Yeni Harita Ekle:")
        label_add.setFont(QFont("Arial", 12))
        layout.addWidget(label_add)

        btn_add_map = QPushButton("➕ Harita Ekle")
        btn_add_map.setStyleSheet("color: white; font-weight: bold;")
        btn_add_map.clicked.connect(self.add_new_map)
        layout.addWidget(btn_add_map)

        # Harita sil
        label_delete = QLabel("🗑 Harita Sil:")
        label_delete.setFont(QFont("Arial", 12))
        layout.addWidget(label_delete)

        btn_delete_map = QPushButton("❌ Haritayı Sil")
        btn_delete_map.setStyleSheet("color: white; font-weight: bold;")
        btn_delete_map.clicked.connect(self.delete_selected_map)
        layout.addWidget(btn_delete_map)

        self.setLayout(layout)

        if self.map_paths:
            self.update_map_display(next(iter(self.map_paths)))

    def update_map_display(self, selected_map):
        map_path = self.map_paths.get(selected_map)
        if map_path and os.path.exists(map_path):
            pixmap = QPixmap(map_path)
            scaled = pixmap.scaled(self.map_display.size(), Qt.AspectRatioMode.KeepAspectRatio)
            self.map_display.setPixmap(scaled)
        else:
            self.map_display.setText("Harita dosyası bulunamadı.")

    def send_goal(self):
        self.publisher_target.publish("goal_dummy_sent")
        rospy.loginfo("📍 Hedefe git komutu yayınlandı.")

    def reset_position(self):
        self.publisher_reset.publish("reset_position")
        rospy.loginfo("🔄 Konum sıfırlama komutu yayınlandı.")

    def add_new_map(self):
        file_dialog = QFileDialog()
        file_path, _ = file_dialog.getOpenFileName(self, "Yeni Harita Seç", "", "Image Files (*.ppm *.png *.jpg)")
        if file_path:
            fname = os.path.basename(file_path)
            dest_path = os.path.join(self.image_dir, fname)
            if not os.path.exists(dest_path):
                shutil.copy(file_path, dest_path)
                map_label = os.path.splitext(fname)[0].capitalize().replace("_", " ")
                self.map_paths[map_label] = dest_path
                self.map_selector.addItem(map_label)
                QMessageBox.information(self, "Harita Eklendi", f"{map_label} başarıyla eklendi.")
            else:
                QMessageBox.warning(self, "Zaten Var", "Bu isimde bir harita zaten mevcut.")

    def delete_selected_map(self):
        selected_label = self.map_selector.currentText()
        map_path = self.map_paths.get(selected_label)
        if map_path and os.path.exists(map_path):
            confirm = QMessageBox.question(
                self, "Harita Sil", f"{selected_label} haritasını silmek istediğinize emin misiniz?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
            )
            if confirm == QMessageBox.StandardButton.Yes:
                os.remove(map_path)
                self.map_selector.removeItem(self.map_selector.currentIndex())
                del self.map_paths[selected_label]
                self.map_display.setText("Harita silindi.")



