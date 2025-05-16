import rospy
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QComboBox, QRadioButton, QButtonGroup, QHBoxLayout
from PyQt6.QtGui import QFont
from std_msgs.msg import String

class SettingsPage(QWidget):
    def __init__(self):
        super().__init__()
        self.setup_ui()

    def setup_ui(self):
        layout = QVBoxLayout()

        title = QLabel("⚙️ Ayarlar")
        title.setFont(QFont("Arial", 20))
        layout.addWidget(title)

        # Dil Seçimi
        self.label_language = QLabel("Dil Seçimi:")
        layout.addWidget(self.label_language)

        self.language_selector = QComboBox()
        self.language_selector.addItems(["Türkçe", "English"])
        self.language_selector.currentTextChanged.connect(self.change_language)
        layout.addWidget(self.language_selector)

        # Tema Seçimi
        self.label_theme = QLabel("Tema Seçimi:")
        layout.addWidget(self.label_theme)

        self.theme_selector = QButtonGroup()

        self.radio_light = QRadioButton("Aydınlık Mod")
        self.radio_dark = QRadioButton("Karanlık Mod")
        self.radio_light.setChecked(True)  # Varsayılan tema aydınlık
        self.theme_selector.addButton(self.radio_light)
        self.theme_selector.addButton(self.radio_dark)

        layout.addWidget(self.radio_light)
        layout.addWidget(self.radio_dark)

        # Şarj İstasyonuna Gönderme
        self.btn_send_to_charging = QPushButton("🚗 Robotu Şarj İstasyonuna Gönder")
        self.btn_send_to_charging.clicked.connect(self.send_to_charging_station)
        layout.addWidget(self.btn_send_to_charging)

        # Temizle ve Kaydet Butonları
        button_layout = QHBoxLayout()
        self.btn_save = QPushButton("Kaydet")
        self.btn_save.clicked.connect(self.save_settings)
        button_layout.addWidget(self.btn_save)

        self.btn_cancel = QPushButton("Vazgeç")
        self.btn_cancel.clicked.connect(self.cancel_settings)
        button_layout.addWidget(self.btn_cancel)

        layout.addLayout(button_layout)

        self.setLayout(layout)

    def change_language(self):
        """Dil seçimi yapıldığında yapılacak işlemler"""
        selected_language = self.language_selector.currentText()
        rospy.loginfo(f"Seçilen dil: {selected_language}")
        # Burada ROS ile dil değişikliği yapılabilir, örneğin bir mesaj gönderebiliriz.

    def send_to_charging_station(self):
        """Robotu şarj istasyonuna gönder"""
        rospy.loginfo("Robot şarj istasyonuna gönderildi.")
        # Burada ROS mesajı gönderilebilir, robotun şarj istasyonuna yönlendirilmesi sağlanabilir.

    def save_settings(self):
        """Ayarları kaydet"""
        selected_theme = "Aydınlık Mod" if self.radio_light.isChecked() else "Karanlık Mod"
        rospy.loginfo(f"Seçilen tema: {selected_theme}")
        # Temayı ROS ile değiştirme işlemi yapılabilir.

        # Dil seçimlerini kaydedebiliriz (örneğin, bir dosyaya veya ROS parametrelerine kaydedilebilir)
        selected_language = self.language_selector.currentText()
        rospy.loginfo(f"Seçilen dil kaydedildi: {selected_language}")
        
        # Ayarları kaydetme mesajı
        QMessageBox.information(self, "Ayarlar Kaydedildi", "Ayarlar başarıyla kaydedildi.")

    def cancel_settings(self):
        """Ayarları iptal et"""
        rospy.loginfo("Ayarlar iptal edildi.")
        # İptal edilirse, sayfayı yeniden başlatabiliriz veya eski ayarlara dönebiliriz.
        self.close()
