import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import boot_serial
import boot_cmd

class MainApp(QMainWindow):

    def __init__(self, parent=None):
        super(MainApp, self).__init__(parent)

        self.setWindowTitle("Bootloader Host Application")
        self.setFixedSize(500, 500)

        widget = QWidget()

        grid = QGridLayout()
        self.btn_grp_cnt = CntBtnGrp("Connection:")
        grid.addWidget(self.btn_grp_cnt, 0, 0, 1, 2)
        self.btn_grp_cmd = CmdBtnGrp("Commands:")
        grid.addWidget(self.btn_grp_cmd, 1, 0)
        self.display = Display("Display:")
        grid.addWidget(self.display, 1, 1)
        widget.setLayout(grid)
        self.setCentralWidget(widget)

        # Connections
        self.btn_grp_cnt.btn_connect.clicked.connect(self.slot_connect)
        self.btn_grp_cmd.btn_cmd_ver.clicked.connect(self.slot_version)
        self.btn_grp_cmd.btn_cmd_help.clicked.connect(self.slot_help)
        self.btn_grp_cmd.btn_cmd_cid.clicked.connect(self.slot_cid)
        self.btn_grp_cmd.btn_cmd_rdp.clicked.connect(self.slot_rdp)

    def slot_connect(self):
        if boot_serial.connect_serial(self.btn_grp_cnt.usb_list.currentText()):
            self.btn_grp_cmd.setEnabled(True)
        else:
            pass

    def slot_version(self):
        self.display.lab_display.setText('Bootloader Version: ' + hex(boot_cmd.cmd_ver()[0]))

    def slot_help(self):
        value = boot_cmd.cmd_help()
        str_cmp = []
        str_cmp.append('Commands Supported:\n')
        for x in value:
            str_cmp.append('\n' + hex(x))
        self.display.lab_display.setText(''.join(str_cmp))

    def slot_cid(self):
        value = boot_cmd.cmd_cid()
        str_cmp = []
        str_cmp.append('Chip Identifier: ')
        ci = (value[1] << 8 )+ value[0]
        str_cmp.append(hex(ci))
        self.display.lab_display.setText(''.join(str_cmp))

    def slot_rdp(self):
        self.display.lab_display.setText('Read Protection Opt Lvl: ' + hex(boot_cmd.cmd_rdp()[0]))

class CntBtnGrp(QGroupBox):

    def __init__(self, name):
        super(CntBtnGrp, self).__init__(name)

        self.btn_connect = QPushButton("Connect")
        self.usb_list = QComboBox()
        self.usb_list.addItems(boot_serial.serial_ports())

        hbox = QHBoxLayout()
        hbox.addWidget(self.btn_connect)
        hbox.addWidget(self.usb_list)
        hbox.addStretch(1)
        self.setLayout(hbox)

class CmdBtnGrp(QGroupBox):

    def __init__(self, name):
        super(CmdBtnGrp, self).__init__(name)

        self.btn_cmd_ver = QPushButton("Get Version")
        self.btn_cmd_help = QPushButton("Get Help")
        self.btn_cmd_cid = QPushButton("Get CID")
        self.btn_cmd_rdp = QPushButton("Get RDP")
        self.btn_cmd_go = QPushButton("Go Addr")
        self.btn_cmd_erase = QPushButton("Erase")
        self.btn_cmd_write = QPushButton("Write")
        self.btn_cmd_read_sector_status = QPushButton("Read Sector Status")
        self.setEnabled(False)

        vbox = QVBoxLayout()
        vbox.addWidget(self.btn_cmd_ver)
        vbox.addWidget(self.btn_cmd_help)
        vbox.addWidget(self.btn_cmd_cid)
        vbox.addWidget(self.btn_cmd_rdp)
        vbox.addWidget(self.btn_cmd_go)
        vbox.addWidget(self.btn_cmd_erase)
        vbox.addWidget(self.btn_cmd_write)
        vbox.addWidget(self.btn_cmd_read_sector_status)
        vbox.addStretch(1)
        self.setLayout(vbox)

class Display(QGroupBox):

    def __init__(self, name):
        super(Display, self).__init__(name)

        self.lab_display = QLabel()
        self.lab_display.setAlignment(Qt.AlignTop | Qt.AlignLeft)

        vbox = QVBoxLayout()
        vbox.addWidget(self.lab_display)
        vbox.addStretch(1)
        self.setLayout(vbox)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainApp()
    window.show()
    sys.exit(app.exec_())
