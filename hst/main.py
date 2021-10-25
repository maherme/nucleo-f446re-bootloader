import sys
import os
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import QtCore
import boot_serial
import boot_cmd

file_name = ""
file_size = 0
address = 0
protection_mode = ["Write Protection", "Read/Write Protection","No protection"]

class MainApp(QMainWindow):
    def __init__(self, parent=None):
        super(MainApp, self).__init__(parent)

        self.setWindowTitle("Bootloader Host Application")
        #self.setFixedSize(500, 500)

        widget = QWidget()

        grid = QGridLayout()
        self.btn_grp_cnt = CntBtnGrp("Connection:")
        grid.addWidget(self.btn_grp_cnt, 0, 0, 1, 2)
        self.btn_grp_cmd = CmdBtnGrp("Commands:")
        grid.addWidget(self.btn_grp_cmd, 1, 0)
        self.display = Display("Display:")
        self.erase_menu = EraseMenu("Erase Menu:")
        self.write_display = WriteDisplay("Write Memory:")
        self.rw_prot_display = RWProtectDisplay("Read/Write Protection Menu:")
        self.read_display = MemReadDisplay("Read Memory:")
        self.stack_lay = QStackedLayout()
        self.stack_lay.addWidget(self.display)
        self.stack_lay.addWidget(self.erase_menu)
        self.stack_lay.addWidget(self.write_display)
        self.stack_lay.addWidget(self.rw_prot_display)
        self.stack_lay.addWidget(self.read_display)
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
        self.btn_grp_cmd.btn_cmd_write.clicked.connect(self.slot_write)
        self.write_display.btn_start.clicked.connect(self.slot_write_start)
        self.btn_grp_cmd.btn_cmd_read_sector_status.clicked.connect(self.slot_read_sector_st)
        self.btn_grp_cmd.btn_cmd_rw_protect.clicked.connect(self.slot_rw_protect)
        self.rw_prot_display.check_dis_protect.stateChanged.connect(self.slot_rw_disable)
        self.rw_prot_display.btn_write_protect.clicked.connect(self.slot_rw_protect_write)
        self.btn_grp_cmd.btn_cmd_read.clicked.connect(self.slot_read)
        self.read_display.btn_read.clicked.connect(self.slot_read_start)

    def slot_connect(self):
        if boot_serial.connect_serial(self.btn_grp_cnt.usb_list.currentText()):
            self.btn_grp_cmd.setEnabled(True)
        else:
            pass

    def slot_version(self):
        self.stack_lay.setCurrentIndex(0)
        self.display.lab_display.setText('Bootloader Version: ' + hex(boot_cmd.cmd_ver()[0]))

    def slot_help(self):
        self.stack_lay.setCurrentIndex(0)
        value = boot_cmd.cmd_help()
        str_cmp = []
        str_cmp.append('Commands Supported:\n')
        for x in value:
            str_cmp.append('\n' + hex(x))
        self.display.lab_display.setText(''.join(str_cmp))

    def slot_cid(self):
        self.stack_lay.setCurrentIndex(0)
        value = boot_cmd.cmd_cid()
        str_cmp = []
        str_cmp.append('Chip Identifier: ')
        ci = (value[1] << 8 ) + value[0]
        str_cmp.append(hex(ci))
        self.display.lab_display.setText(''.join(str_cmp))

    def slot_rdp(self):
        self.stack_lay.setCurrentIndex(0)
        self.display.lab_display.setText('Read Protection Opt Lvl: ' + hex(boot_cmd.cmd_rdp()[0]))

    def slot_go(self):
        self.stack_lay.setCurrentIndex(0)
        text, ok = QInputDialog().getText(self, \
                                          'Enter address to jump:', \
                                          'Address (hex format):', \
                                          QLineEdit.Normal, \
                                          '0x08008346')
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
                msg_sector_erase = QMessageBox()
                msg_sector_erase.setWindowTitle("Information Message")
                msg_sector_erase.setIcon(QMessageBox.Information)
                if(offset == 1):
                    msg_sector_erase.setText("Sector " + str(sector) + " will be erased")
                else:
                    msg_sector_erase.setText("Sectors from " + \
                                             str(sector) + \
                                             " to " + \
                                             str(sector + offset - 1) + \
                                             " will be erased")
                msg_sector_erase.setInformativeText("Are you sure to continue?")
                msg_sector_erase.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
                msg_sector_erase.buttonClicked.connect(self.slot_sector_erase_ok)
                msg_sector_erase.exec_()

    def slot_mass_erase_ok(self, i):
        if(i.text() == 'OK'):
            boot_cmd.cmd_erase(0xFF, 0)
        else:
            pass

    def slot_sector_erase_ok(self, i):
        if(i.text() == 'OK'):
            boot_cmd.cmd_erase(self.erase_menu.spin_sector.value(), self.erase_menu.spin_num_sector.value())
        else:
            pass

    def slot_write(self):
        global file_name
        global file_size
        global address

        self.stack_lay.setCurrentIndex(2)
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        dlg = QFileDialog()
        file_name, _filter = dlg.getOpenFileName(None, \
                                                 'Select Binary File', \
                                                 './', \
                                                 filter = '*.bin', \
                                                 options = options)
        if file_name:
            file_size = os.path.getsize(file_name)
            address, ok = QInputDialog().getText(self, \
                                                 'Enter Starting Address to Write:', \
                                                 'Address (hex format):', \
                                                 QLineEdit.Normal, \
                                                 '0x08008000')
            if ok and address:
                self.write_display.lab_file.setText("File: " + file_name)
                self.write_display.lab_size.setText("Size: " + str(file_size) + " Bytes")
                self.write_display.lab_addr.setText("Starting Address: " + address)

    def slot_write_start(self):
        global address

        address_flash = int(address, 16)
        bytes_remaining = 0
        bytes_to_sent = 0
        len_to_read = 0

        bytes_remaining = file_size - bytes_to_sent

        bin_file = open(file_name,'rb')

        while(bytes_remaining):
            if(bytes_remaining >= 128):
                len_to_read = 128
            else:
                len_to_read = bytes_remaining

            boot_cmd.cmd_write(len_to_read, address_flash, bin_file)

            address_flash += len_to_read
            bytes_to_sent += len_to_read
            bytes_remaining = file_size - bytes_to_sent

            self.write_display.pbar.setValue((bytes_to_sent/file_size)*100)

    def slot_read_sector_st(self):
        self.stack_lay.setCurrentIndex(0)
        value = boot_cmd.cmd_read_sector_st()

        if(value[1] & (1 << 7)):
            self.display.lab_display.setText('Flash Protection Mode: Read/Write Protection (PCROP)\n')
        else:
            self.display.lab_display.setText('Flash Protection Mode: Write Protection\n')

        for x in range(8):
            self.display.lab_display.setText(self.display.lab_display.text() + \
                                             "\nSector " + \
                                             str(x) + \
                                             ":\t" + \
                                             protect_type(value, x))

    def slot_rw_protect(self):
        self.stack_lay.setCurrentIndex(3)

    def slot_rw_disable(self, state):
        if(QtCore.Qt.Checked == state):
            self.rw_prot_display.check_sprmod.setEnabled(False)
            self.rw_prot_display.check_sector0.setEnabled(False)
            self.rw_prot_display.check_sector1.setEnabled(False)
            self.rw_prot_display.check_sector2.setEnabled(False)
            self.rw_prot_display.check_sector3.setEnabled(False)
            self.rw_prot_display.check_sector4.setEnabled(False)
            self.rw_prot_display.check_sector5.setEnabled(False)
            self.rw_prot_display.check_sector6.setEnabled(False)
            self.rw_prot_display.check_sector7.setEnabled(False)
        else:
            self.rw_prot_display.check_sprmod.setEnabled(True)
            self.rw_prot_display.check_sector0.setEnabled(True)
            self.rw_prot_display.check_sector1.setEnabled(True)
            self.rw_prot_display.check_sector2.setEnabled(True)
            self.rw_prot_display.check_sector3.setEnabled(True)
            self.rw_prot_display.check_sector4.setEnabled(True)
            self.rw_prot_display.check_sector5.setEnabled(True)
            self.rw_prot_display.check_sector6.setEnabled(True)
            self.rw_prot_display.check_sector7.setEnabled(True)

    def slot_rw_protect_write(self):
        mode = 1
        sectors = 0

        if self.rw_prot_display.check_dis_protect.isChecked():
            boot_cmd.cmd_dis_rw_protect()
        else:
            if self.rw_prot_display.check_sprmod.isChecked():
                mode = 2

            if self.rw_prot_display.check_sector0.isChecked():
                sectors |= 0x01
            if self.rw_prot_display.check_sector1.isChecked():
                sectors |= 0x02
            if self.rw_prot_display.check_sector2.isChecked():
                sectors |= 0x04
            if self.rw_prot_display.check_sector3.isChecked():
                sectors |= 0x08
            if self.rw_prot_display.check_sector4.isChecked():
                sectors |= 0x10
            if self.rw_prot_display.check_sector5.isChecked():
                sectors |= 0x20
            if self.rw_prot_display.check_sector6.isChecked():
                sectors |= 0x40
            if self.rw_prot_display.check_sector7.isChecked():
                sectors |= 0x80

            boot_cmd.cmd_en_rw_protect(sectors, mode)

    def slot_read(self):
        self.stack_lay.setCurrentIndex(4)

    def slot_read_start(self):
        self.read_display.lab_mem_read.clear()
        addr = int(self.read_display.text_addr.text(), 16)
        offset = int(self.read_display.text_offset.text(), 10)
        if(4*offset < 256):
            value = boot_cmd.cmd_mem_read(addr, 4*offset)
            if(value[0] == 0):
                for i,_ in enumerate(value[1:]):
                    i += 1
                    if(not i%4):
                        mem_read_value = (value[i] << 24) + \
                                         (value[i-1] << 16) + \
                                         (value[i-2] << 8) + \
                                         value[i-3]
                        self.read_display.lab_mem_read.setText(self.read_display.lab_mem_read.text() + \
                                                               '0x{0:0{1}X}'.format(addr + i - 4, 8) + \
                                                               "\t" + \
                                                               '0x{0:0{1}X}'.format(mem_read_value, 8) + \
                                                               "\n")
            else:
                self.read_display.lab_mem_read.setText("Error: try accessing to an invalid memory address!")
        else:
            self.read_display.lab_mem_read.setText("Error: too size for offset value!")

def protect_type(status, n):
    if(status[1] & (1 << 7)):
        if(status[0] & (1 << n)):
            return protection_mode[1]
        else:
            return protection_mode[2]
    else:
        if(status[0] & (1 << n)):
            return protection_mode[2]
        else:
            return protection_mode[0]

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
        self.btn_cmd_rw_protect = QPushButton("R/W Protection")
        self.btn_cmd_read = QPushButton("Read")
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
        vbox.addWidget(self.btn_cmd_rw_protect)
        vbox.addWidget(self.btn_cmd_read)
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

class WriteDisplay(QGroupBox):
    def __init__(self, name):
        super(WriteDisplay, self).__init__(name)

        self.lab_file = QLabel("File:")
        self.lab_size = QLabel("Size:")
        self.lab_addr = QLabel("Starting Address:")
        self.lab_pbar = QLabel("Progress:")
        self.pbar = QProgressBar()
        self.pbar.setMaximum(100)
        self.pbar.setFixedWidth(600)
        self.btn_start = QPushButton("Start")
        self.btn_start.setFixedWidth(120)

        vbox = QVBoxLayout()
        vbox.addWidget(self.lab_file)
        vbox.addWidget(self.lab_size)
        vbox.addWidget(self.lab_addr)
        vbox.addWidget(self.lab_pbar)
        vbox.addWidget(self.pbar)
        vbox.addWidget(self.btn_start)
        vbox.addStretch(1)
        self.setLayout(vbox)

class RWProtectDisplay(QGroupBox):
    def __init__(self, name):
        super(RWProtectDisplay, self).__init__(name)

        self.check_dis_protect = QCheckBox("Disable R/W Protection for All Sectors")
        self.check_sprmod = QCheckBox("Set SPRMOD Bit in OPTCR Flash Register (Set for R/W Protection)")
        self.lab_info = QLabel("Select Sectors to Apply Protection:")
        self.check_sector0 = QCheckBox("Sector 0")
        self.check_sector1 = QCheckBox("Sector 1")
        self.check_sector2 = QCheckBox("Sector 2")
        self.check_sector3 = QCheckBox("Sector 3")
        self.check_sector4 = QCheckBox("Sector 4")
        self.check_sector5 = QCheckBox("Sector 5")
        self.check_sector6 = QCheckBox("Sector 6")
        self.check_sector7 = QCheckBox("Sector 7")
        self.btn_write_protect = QPushButton("Write")
        self.btn_write_protect.setFixedWidth(120)

        vbox = QVBoxLayout()
        vbox.addWidget(self.check_dis_protect)
        vbox.addWidget(self.check_sprmod)
        vbox.addWidget(self.lab_info)
        vbox.addWidget(self.check_sector0)
        vbox.addWidget(self.check_sector1)
        vbox.addWidget(self.check_sector2)
        vbox.addWidget(self.check_sector3)
        vbox.addWidget(self.check_sector4)
        vbox.addWidget(self.check_sector5)
        vbox.addWidget(self.check_sector6)
        vbox.addWidget(self.check_sector7)
        vbox.addWidget(self.btn_write_protect)
        vbox.addStretch(1)
        self.setLayout(vbox)

class MemReadDisplay(QGroupBox):
    def __init__(self, name):
        super(MemReadDisplay, self).__init__(name)

        self.lab_addr = QLabel("Base Address")
        self.text_addr = QLineEdit(self)
        self.text_addr.setFixedWidth(120)
        self.lab_offset = QLabel("Length (in word)")
        self.text_offset = QLineEdit(self)
        self.text_offset.setFixedWidth(120)
        self.btn_read = QPushButton("Read")
        self.btn_read.setFixedWidth(120)
        self.lab_mem_read = QLabel()
        self.lab_mem_read.setAlignment(Qt.AlignTop | Qt.AlignLeft)

        self.scroll = QScrollArea()
        self.scroll.setWidget(self.lab_mem_read)
        self.scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scroll.setWidgetResizable(True)
        self.scroll.setFixedHeight(350)

        hbox = QHBoxLayout()
        hbox.addWidget(self.lab_addr)
        hbox.addWidget(self.text_addr)
        hbox.addWidget(self.lab_offset)
        hbox.addWidget(self.text_offset)
        hbox.addWidget(self.btn_read)
        hbox.addStretch(1)
        vbox = QVBoxLayout()
        vbox.addLayout(hbox)
        vbox.addWidget(self.scroll)
        vbox.addStretch(1)
        self.setLayout(vbox)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainApp()
    window.show()
    sys.exit(app.exec_())
