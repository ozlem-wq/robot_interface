import rospy
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QComboBox, QRadioButton, QButtonGroup, QMessageBox
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont

class SettingsPage(QWidget):
    def __init__(self):
        super().__init__()
        self.setObjectName("settingsPage")
        self.setup_ui()

    def setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(50, 30, 50, 30)

        title = QLabel("⚙ Ayarlar")
        title.setFont(QFont("Arial", 20, QFont.Weight.Bold))
        title.setStyleSheet("color: white;")
        layout.addWidget(title)

        # Dil seçimi
        self.label_language = QLabel("🗣️ Dil Seçimi:")
        self.label_language.setStyleSheet("color: #55aa55; font-weight: bold;")
        layout.addWidget(self.label_language)

        self.language_selector = QComboBox()
        self.language_selector.addItems(["Türkçe", "English"])
        layout.addWidget(self.language_selector)

        # Tema seçimi
        self.label_theme = QLabel("🎨 Tema Seçimi:")
        self.label_theme.setStyleSheet("color: #55aa55; font-weight: bold;")
        layout.addWidget(self.label_theme)

        self.radio_light = QRadioButton("Aydınlık Mod")
        self.radio_dark = QRadioButton("Karanlık Mod")
        self.radio_light.setChecked(True)
        layout.addWidget(self.radio_light)
        layout.addWidget(self.radio_dark)

        self.theme_group = QButtonGroup()
        self.theme_group.addButton(self.radio_light)
        self.theme_group.addButton(self.radio_dark)

        # Robotu şarj istasyonuna gönder
        self.btn_send_to_charging = QPushButton("🔌 Robotu Şarj İstasyonuna Gönder")
        self.btn_send_to_charging.clicked.connect(self.send_to_charging)
        layout.addWidget(self.btn_send_to_charging)

        # Kaydet
        self.btn_save = QPushButton("💾 Ayarları Kaydet")
        self.btn_save.clicked.connect(self.save_settings)
        layout.addWidget(self.btn_save)

    def send_to_charging(self):
        rospy.loginfo("🔌 Robot şarj istasyonuna gönderiliyor...")

    def save_settings(self):
        language = self.language_selector.currentText()
        theme = "dark" if self.radio_dark.isChecked() else "light"
        rospy.loginfo(f"Seçilen dil: {language}")
        rospy.loginfo(f"Seçilen tema: {'Karanlık Mod' if theme == 'dark' else 'Aydınlık Mod'}")

        QMessageBox.information(self, "Ayarlar Kaydedildi", "Ayarlar başarıyla kaydedildi.")

        # Tema uygulama işlemi
        if theme == "dark":
            with open("ui/style_dark.qss", "r") as f:
                self.setStyleSheet(f.read())
        else:
            with open("ui/style.qss", "r") as f:
                self.setStyleSheet(f.read())

