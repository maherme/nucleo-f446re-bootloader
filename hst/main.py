from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import boot_serial
import boot_cmd

class MainApp(QMainWindow):
    def __init__(self, parent=None, *args):
        super(MainApp, self).__init__(parent=parent)

        self.setFixedSize(500, 400)
        self.setWindowTitle("Bootloader Host Application")

        # Input Widgets
        self.btn_connect = QPushButton("Connect", self)
        self.btn_connect.setGeometry(10, 10, 80, 30)
        self.btn_connect.clicked.connect(self.slot_connect)

        self.usb_list = QComboBox(self)
        self.usb_list.setGeometry(100, 10, 150, 30)
        self.usb_list.addItems(boot_serial.serial_ports())

        self.btn_cmd_ver = QPushButton("Get Version", self)
        self.btn_cmd_ver.setGeometry(10, 50, 150, 30)
        self.btn_cmd_ver.clicked.connect(self.slot_version)

        self.btn_cmd_help = QPushButton("Get Help", self)
        self.btn_cmd_help.setGeometry(10, 90, 150, 30)
        self.btn_cmd_help.clicked.connect(self.slot_help)

        self.btn_cmd_cid = QPushButton("Get CID", self)
        self.btn_cmd_cid.setGeometry(10, 130, 150, 30)
        self.btn_cmd_cid.clicked.connect(self.slot_cid)

        self.btn_cmd_rdp = QPushButton("Get RDP", self)
        self.btn_cmd_rdp.setGeometry(10, 170, 150, 30)
        self.btn_cmd_rdp.clicked.connect(self.slot_rdp)

        self.btn_cmd_go = QPushButton("Go Addr",self)
        self.btn_cmd_go.setGeometry(10, 210, 150, 30)
        self.btn_cmd_go.clicked.connect(self.slot_go)

        self.btn_cmd_erase = QPushButton("Erase", self)
        self.btn_cmd_erase.setGeometry(10, 250, 150, 30)
        self.btn_cmd_erase.clicked.connect(self.slot_erase)

        self.btn_cmd_write = QPushButton("Write", self)
        self.btn_cmd_write.setGeometry(10, 290, 150, 30)
        self.btn_cmd_write.clicked.connect(self.slot_write)

        self.btn_cmd_read_sector_status = QPushButton("Read Sector Status", self)
        self.btn_cmd_read_sector_status.setGeometry(10, 330, 150, 30)
        self.btn_cmd_read_sector_status.clicked.connect(self.slot_read_sector_status)

        # Frame for displays
        self.frame = QFrame(self)
        self.frame.setGeometry(170, 50, 320, 340)
        self.frame.setFrameShape(QFrame.StyledPanel | QFrame.Sunken)

        # Displays
        self.frame.show_ver = QLabel(self)
        self.frame.show_ver.setGeometry(180, 55, 300, 330)
        self.frame.show_ver.setAlignment(Qt.AlignTop | Qt.AlignLeft)

    # Slots
    def slot_connect(self):
        boot_serial.connect_serial(self.usb_list.currentText())

    def slot_version(self):
        self.frame.show_ver.setText('Bootloader Version: ' + hex(boot_cmd.cmd_ver()))

    def slot_help(self):
        pass

    def slot_cid(self):
        pass

    def slot_rdp(self):
        pass

    def slot_go(self):
        pass

    def slot_erase(self):
        pass

    def slot_write(self):
        pass

    def slot_read_sector_status(self):
        pass

if __name__ == '__main__':
    app = QApplication([])
    window = MainApp()
    window.show()
    app.exec_()
