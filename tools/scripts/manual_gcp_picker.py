#!/usr/bin/env python2.7
# manual gcp picker.py

import sys, os, random
from PyQt4 import QtGui, QtCore

class Measurement:
    def __init__(self,first,second):
        self.first = first
        self.second = second

    def draw_points(self,qp,width):
        qp.drawPoint(self.first)
        qp.translate(width,0)
        qp.drawPoint(self.second)
        qp.translate(-width,0)

    def draw_line(self,qp,width):
        qp.drawLine(self.first,self.second+QtCore.QPointF(width,0))

class GCPPicker(QtGui.QWidget):
    def __init__(self,argv):
        super(GCPPicker,self).__init__()

        self.setGeometry(10,10,1000,500)
        self.setWindowTitle('Manual GCP Picker')

        # Produce our temporary images
        #os.system(os.path.realpath(__file__)[:-24]+"libexec/generate_wac_crop " + argv[1] )

        # Load up images
        self.cube_image = QtGui.QImage( argv[1][:-3]+"tif" )
        self.wac_image  = QtGui.QImage( "wac_crop.tif" )
        self.content_original_size = QtCore.QSize( self.cube_image.width() + self.wac_image.width(),
                                                   self.cube_image.height() )
        self.content_scaling = 1

        # Start the list that contains measurement
        self.measurements = []
        self.odd_click = QtCore.QPointF(-1,-1);

    #def __del__(self):
        # Remove our temporary images
        #os.system("rm wac_crop.tif")

    def paintEvent(self, e):
        qp = QtGui.QPainter()
        qp.begin(self)
        qp.scale(self.content_scaling,self.content_scaling)
        qp.drawImage( QtCore.QPoint(0,0), self.cube_image )
        qp.drawImage( QtCore.QPoint(self.cube_image.width(),0), self.wac_image )
        for m in self.measurements:
            color = QtGui.QColor()
            color.setHsv( int(random.uniform(0,255)) ,255,255)
            qp.setPen(color)
            m.draw_line(qp,self.cube_image.width())
        qp.end()

    def resizeEvent(self, e):
        self.content_scaling = min( float(e.size().width()) / float(self.content_original_size.width()),
                                    float(e.size().height()) / float(self.content_original_size.height()) )
        self.update()

    def mousePressEvent(self, e):
        if self.odd_click == QtCore.QPointF(-1,-1):
            self.odd_click = (e.posF() - QtCore.QPointF(1,1)) / self.content_scaling
        else:
            even_click = (e.posF() - QtCore.QPointF(1,1)) / self.content_scaling

            # Here we double check that we have a left and a right
            left_click = QtCore.QPointF(-1,-1)
            right_click = QtCore.QPointF(-1,-1)
            if self.odd_click.x() < self.cube_image.width():
                left_click = self.odd_click
            else:
                right_click = self.odd_click
            self.odd_click = QtCore.QPointF(-1,-1)
            if even_click.x() < self.cube_image.width():
                left_click = even_click
            else:
                right_click = even_click

            if left_click != QtCore.QPointF(-1,-1) and right_click != QtCore.QPointF(-1,-1):
                self.measurements.append( Measurement(left_click,right_click - QtCore.QPointF(self.cube_image.width(),0) ) )
                self.update()

    def keyPressEvent(self, e):
        if e.key() == QtCore.Qt.Key_D:
            self.measurements.pop()
            self.update()

if not sys.argv[1:]:
    print "Usage: python manual_gcp_picker.py cubefile"

app = QtGui.QApplication(sys.argv)
window = GCPPicker(sys.argv)
window.show()
app.exec_()
