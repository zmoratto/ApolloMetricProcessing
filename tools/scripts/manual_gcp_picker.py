#!/usr/bin/env python2.7
# manual gcp picker.py

import sys, os
from PyQt4 import QtGui, QtCore

class GCPPicker(QtGui.QWidget):
    def __init__(self,argv):
        super(GCPPicker,self).__init__()

        self.setGeometry(10,10,1000,500)
        self.setWindowTitle('Manual GCP Picker')

        # Produce our temporary images
        os.system(os.path.realpath(__file__)[:-19]+"libexec/generate_wac_crop " + argv[0] );

        # Load up images
        self.cube_image = QtGui.QImage( argv[0][:-3]+"tif" )
        self.wac_image  = QtGui.QImage( "wac_crop.tif" )

    def __del__(self):
        # Remove our temporary images
        os.system("rm wac_crop.tif")

        super(GCPPicker,self).__init__()

    def paintEvent(self, e):
        qp = QtGui.QPainter()
        qp.begin(self)
        qp.drawImage( QtCore.QPoint(0,0), self.cube_image )
        qp.drawImage( QtCore.QPoint(self.cube_image.width(),0), self.wac_image )
        qp.end()

if not sys.argv[1:]:
    print "Usage: python manual_gcp_picker.py cubefile"

app = QtGui.QApplication(sys.argv)
window = GCPPicker(sys.argv)
window.show()
app.exec_()
