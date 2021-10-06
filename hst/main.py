import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import QtCore
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
        self.erase_menu = EraseMenu("Erase Menu:")
        self.stack_lay = QStackedLayout()
        self.stack_lay.addWidget(self.display)
        self.stack_lay.addWidget(self.erase_menu)
        grid.addLayout(self.stack_lay, 1, 1, 1, 2)
        widget.setLayout(grid)
        self.setCentralWidget(widget)

        # Connections
        self.btn_grp_cnt.btn_connect.clicked.connect(self.slot_connect)
        self.btn_grp_cmd.btn_cmd_ver.clicked.connect(self.slot_version)
        self.btn_grp_cmd.btn_cmd_help.clicked.connect(self.slot_help)
        self.btn_grp_cmd.btn_cmd_cid.clicked.connect(self.slot_cid)
        self.btn_grp_cmd.btn_cmd_rdp.clicked.connect(self.slot_rdp)
        self.btn_grp_cmd.btn_cmd_go.clicked.connect(self.slot_go)
        self.btn_grp_cmd.btn_cmd_erase.clicked.connect(self.slot_erase)
        self.erase_menu.btn_cancel.clicked.connect(self.slot_cancel_erase)
        self.erase_menu.check_mass_erase.stateChanged.connect(self.slot_mass_erase)
        self.erase_menu.btn_ok.clicked.connect(self.slot_ok_erase)

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
        ci = (value[1] << 8 ) + value[0]
        str_cmp.append(hex(ci))
        self.display.lab_display.setText(''.join(str_cmp))

    def slot_rdp(self):
        self.display.lab_display.setText('Read Protection Opt Lvl: ' + hex(boot_cmd.cmd_rdp()[0]))

    def slot_go(self):
        text, ok = QInputDialog().getText(self, 'Enter address to jump:', 'Address (hex format):', QLineEdit.Normal, '0x08008346')
        if ok and text:
            ret = boot_cmd.cmd_go(text)
            if ret[0] == 0:
                str_cmp = []
                str_cmp.append('Jump to address: ')
                str_cmp.append(text)
                self.display.lab_display.setText(''.join(str_cmp))
            else:
                self.display.lab_display.setText('ERROR: Invalid Address')

    def slot_erase(self):
        self.btn_grp_cmd.setEnabled(False)
        self.stack_lay.setCurrentIndex(1)

    def slot_mass_erase(self, state):
        if(QtCore.Qt.Checked == state):
            self.erase_menu.spin_sector.setEnabled(False)
            self.erase_menu.spin_num_sector.setEnabled(False)
        else:
            self.erase_menu.spin_sector.setEnabled(True)
            self.erase_menu.spin_num_sector.setEnabled(True)

    def slot_cancel_erase(self):
        self.stack_lay.setCurrentIndex(0)
        self.btn_grp_cmd.setEnabled(True)

    def slot_ok_erase(self):
        if self.erase_menu.check_mass_erase.isChecked():
            msg_mass_erase = QMessageBox()
            msg_mass_erase.setWindowTitle("Warning Message")
            msg_mass_erase.setIcon(QMessageBox.Warning)
            msg_mass_erase.setText("A Mass Erase Will Be Done!!!")
            msg_mass_erase.setInformativeText("Are you sure to continue?")
            msg_mass_erase.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
            msg_mass_erase.buttonClicked.connect(self.slot_mass_erase_ok)
            msg_mass_erase.exec_()
        else:
            sector = self.erase_menu.spin_sector.value()
            offset = self.erase_menu.spin_num_sector.value()
            if((sector + offset) > 8):
                msg_error_erase = QMessageBox()
                msg_error_erase.setWindowTitle("Error Message")
                msg_error_erase.setIcon(QMessageBox.Critical)
                msg_error_erase.setText("Invalid Number of Sectors")
                msg_error_erase.setStandardButtons(QMessageBox.Ok)
                msg_error_erase.exec_()
            else:
                msg_mass_erase = QMessageBox()
                msg_mass_erase.setWindowTitle("Information Message")
                msg_mass_erase.setIcon(QMessageBox.Information)
                if(offset == 1):
                    msg_mass_erase.setText("Sector " + str(sector) + " will be erased")
                else:
                    msg_mass_erase.setText("Sectors from " + str(sector) + " to " + str(sector + offset - 1) + " will be erased")
                msg_mass_erase.setInformativeText("Are you sure to continue?")
                msg_mass_erase.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
                msg_mass_erase.buttonClicked.connect(self.slot_sector_erase_ok)
                msg_mass_erase.exec_()

    def slot_mass_erase_ok(self, i):
        if(i.text() == 'OK'):
            print('mass_erase')
        else:
            pass

    def slot_sector_erase_ok(self, i):
        if(i.text() == 'OK'):
            print('sector erase')
        else:
            pass

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

class EraseMenu(QGroupBox):

    def __init__(self, name):
        super(EraseMenu, self).__init__(name)

        self.check_mass_erase = QCheckBox("Mass Erase")
        self.label1 = QLabel("Starting Sector to Erase:")
        self.spin_sector = QSpinBox()
        self.spin_sector.setFixedWidth(60)
        self.spin_sector.setRange(0, 7)
        self.label2 = QLabel("Number of Consecutive Sectors to Erase:")
        self.spin_num_sector = QSpinBox()
        self.spin_num_sector.setRange(1, 8)
        self.spin_num_sector.setFixedWidth(60)
        self.btn_ok = QPushButton("OK")
        self.btn_ok.setFixedWidth(120)
        self.btn_cancel = QPushButton("Cancel")
        self.btn_cancel.setFixedWidth(120)

        vbox = QVBoxLayout()
        hbox = QHBoxLayout()
        vbox.addWidget(self.check_mass_erase)
        vbox.addWidget(self.label1)
        vbox.addWidget(self.spin_sector)
        vbox.addWidget(self.label2)
        vbox.addWidget(self.spin_num_sector)
        hbox.addWidget(self.btn_ok)
        hbox.addWidget(self.btn_cancel)
        hbox.addStretch(1)
        vbox.addLayout(hbox)
        vbox.addStretch(1)
        self.setLayout(vbox)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainApp()
    window.show()
    sys.exit(app.exec_())
