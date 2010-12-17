#!/opt/local/bin/python2.6

from Tkinter import *
import Image, ImageTk, sys, os
from numpy import *
from struct import *

def solve_euclidean(meas1, meas2):
    mean1 = array(meas1).sum(0) / 2.0
    mean2 = array(meas2).sum(0) / 2.0

    H = array([meas1[0]-mean1]).transpose()*(meas2[0]-mean2)
    H = H + array([meas1[1]-mean1]).transpose()*(meas2[1]-mean2)

    U, S, Vt = linalg.svd(H)
    rotation = dot(transpose(Vt),transpose(U))
    translation = mean2-dot(rotation,mean1)
    output = identity(3,float)
    output[0:2,0:2] = rotation
    output[0:2,2] = translation
    return output

def solve_affine(meas1, meas2):
    y = zeros((len(meas1)*2,1),float)
    A = zeros((len(meas1)*2,6),float)

    for i in range(0,len(meas1)):
        for j in range(0,2):
            row = i*2+j
            A[row,0+j*3:2+j*3] = meas1[i]
            A[row,2+j*3] = 1
            y[row] = meas2[i][j]

    x = linalg.lstsq(A,y)[0]

    solution = identity(3,float)
    solution[0,0:3] = x[0:3,0]
    solution[1,0:3] = x[3:6,0]
    return solution

def solve_homography(meas1, meas2):
    print "Hi"

class ReducedImage:
    def __init__(self, photoimage, scale):
        self.photoimage = photoimage
        self.scale = scale

class App:
    def __init__(self):
        self.root = Tk()
        self.obj_height = self.root.winfo_screenheight()
        self.obj_width  = self.root.winfo_screenwidth()/2
        self.last_click = [-1,-1]
        self.__tmp_objects = []
        self.__match_draw_objects = []
        self.__match_draw_mode = 0
        self.measurement1 = []
        self.measurement2 = []
        self.loaded_measurement1 = []
        self.loaded_measurement2 = []
        self.transform = []

        # Loading up input images
        self.image1 = self.load_image( sys.argv[1] )
        self.image2 = self.load_image( sys.argv[2] )

        # Creating canvas, and drawing
        self.canvas = Canvas(self.root,
                             width=self.obj_width*2,
                             height=self.image1.photoimage.height())
        self.canvas.create_image(1,1,
                                 image=self.image1.photoimage,anchor=NW)
        self.canvas.create_image(1+self.obj_width,1,
                                 image=self.image2.photoimage,anchor=NW)
        self.canvas.bind("<Button-1>",self.button1_click)
        self.root.bind("<Key>",self.key_press)

        # Check to see if a match file exists
        match = sys.argv[1][:sys.argv[1].rfind(".")] + "__" + sys.argv[2][:sys.argv[2].rfind(".")] + ".match"
        print "Match: ", match
        if ( os.path.exists(match) ):
            self.load_match_file( match )
            self.draw_loaded_matches()

        self.canvas.pack()

    def read_ip( self, file ):
        # x = 4f,   y = 4f, ix = 4i, iy = 4i
        # ori = 4f, s = 4f, in = 4f, bool = 1
        # oc = 4u, sc = 4u, size = 8u, float array
        ip_front_raw = file.read(29)
        ip_back_raw  = file.read(16)
        ip_front = unpack('ffiifff?',ip_front_raw)
        ip_back  = unpack('IIQ',ip_back_raw)
        file.read(ip_back[2]*4)
        return array([ip_front[0], ip_front[1]])

    def load_match_file(self, match_file):
        print "Reading: ", match_file
        file = open(match_file,"rb")
        ip1_size_raw = file.read(8)
        ip2_size_raw = file.read(8)
        ip1_size = unpack('Q',ip1_size_raw)[0]
        ip2_size = unpack('Q',ip2_size_raw)[0]
        try:
            for i in range(0,ip1_size):
                self.loaded_measurement1.append(self.image1.scale*self.read_ip(file))
            for i in range(0,ip2_size):
                self.loaded_measurement2.append(self.image2.scale*self.read_ip(file))
        finally:
            file.close()

    def draw_loaded_matches(self):
        for i in self.__match_draw_objects:
            self.canvas.delete(i)
        if ( self.__match_draw_mode == 0 ):
            # Draw lines
            for i in range(0,len(self.loaded_measurement1)):
                self.__match_draw_objects.append(self.canvas.create_line(self.loaded_measurement1[i][0],self.loaded_measurement1[i][1],self.loaded_measurement2[i][0]+self.obj_width,self.loaded_measurement2[i][1], fill="red") )
        elif ( self.__match_draw_mode == 1 ):
            # Draw dots
            for i in range(0,len(self.loaded_measurement1)):
                self.__match_draw_objects.append(self.draw_circle(self.loaded_measurement1[i],2,"red"))
                self.__match_draw_objects.append(self.draw_circle(self.loaded_measurement2[i]+array([self.obj_width,0]),2,"red"))

    def load_image(self, image_name):
        # This loads up and image and then scales it
        im = Image.open(image_name)
        [im_w, im_h] = im.size
        im_scale = float(self.obj_width)/float(im_w)
        im = im.resize([im_scale*im_w,im_scale*im_h])
        imtk = ImageTk.PhotoImage( im, palette=256 )
        return ReducedImage(imtk, im_scale)

    def draw_circle( self, center, radius, color ):
        return self.canvas.create_oval(center[0]-radius,center[1]-radius,
                                       center[0]+radius,center[1]+radius,
                                       outline=color)

    def add_measurement(self, coord_a, coord_b ):
        # This will record a measurement and draw the successful match
        left = []
        right = []
        if ( coord_a[0] < coord_b[0] ):
            left = coord_a
            right = coord_b
        else:
            left = coord_b
            right = coord_a
        self.draw_circle(left,1,"green")
        self.draw_circle(right,1,"green")
        right[0] = right[0] - self.obj_width
        self.measurement1.append(array(left))
        self.measurement2.append(array(right))

    def predict_next_location(self):
        measurements = len(self.measurement1)
        transform = identity(3,float)
        if ( measurements == 0 ):
            # Can't do anythin
            return
        elif ( measurements == 1 ):
            transform[0][2] = self.measurement2[0][0] - self.measurement1[0][0]
            transform[1][2] = self.measurement2[0][1] - self.measurement1[0][1]
        elif ( measurements == 2 ):
            transform = solve_euclidean(self.measurement1,
                                        self.measurement2)
        else:
            transform = solve_affine(self.measurement1,
                                     self.measurement2)
        prediction = array([0,0])
        if ( self.last_click[0] >= self.obj_width ):
            # Run backwards
            itransform = linalg.pinv( transform )
            prediction = dot(itransform,array([self.last_click[0]-self.obj_width,self.last_click[1],1]));
            prediction = prediction/prediction[2]
        else:
            # Run forwards
            prediction = dot(transform,array([self.last_click[0],self.last_click[1],1]));
            prediction = prediction/prediction[2]
            prediction[0] = prediction[0] + self.obj_width
        prediction = array([prediction[0],prediction[1]])
        print prediction;
        if ( measurements < 4 ):
            self.__tmp_objects.append(self.draw_circle(prediction,100-measurements*25,"yellow"))
        else:
            self.__tmp_objects.append(self.draw_circle(prediction,20,"yellow"))

    def key_press(self, event):
        if event.char == 's' or event.char == 'S':
            output = sys.argv[1][:sys.argv[1].rfind(".")] + "__" + sys.argv[2][:sys.argv[2].rfind(".")] + ".csv"
            f = open(output, 'w')
            for i in range(0,len(self.measurement1)):
                f.write(str(self.measurement1[i][0]/self.image1.scale)+","+str(self.measurement1[i][1]/self.image1.scale)+","+str(self.measurement2[i][0]/self.image2.scale)+","+str(self.measurement2[i][1]/self.image2.scale)+"\n")
            f.close()
        elif event.char == 'q' or event.char == 'Q':
            print "Quit!"
            sys.exit()
        elif event.char == 'm' or event.char == 'M':
            self.__match_draw_mode = (self.__match_draw_mode + 1 ) % 2
            self.draw_loaded_matches()
        elif event.char == 'r' or event.char == 'R':
            self.loaded_measurement1 = []
            self.loaded_measurement2 = []
            cmd = "ip_guided_match ["+str(self.transform[0][0])+","+str(self.transform[0][1])+","+str(self.transform[0][2])+","+str(self.transform[1][0])+","+str(self.transform[1][1])+","+str(self.transform[1][2])+","+str(self.transform[2][0])+","+str(self.transform[2][1])+","+str(self.transform[2][2])+"] "+sys.argv[1]+" "+sys.argv[2]+" --pass1 400"
            print cmd
            os.system(cmd)
            match = sys.argv[1][:sys.argv[1].rfind(".")] + "__" + sys.argv[2][:sys.argv[2].rfind(".")] + ".match"
            print "Match: ", match
            if ( os.path.exists(match) ):
                self.load_match_file( match )
                self.draw_loaded_matches()

    def button1_click(self, event):
        # Button 1 callback
        if self.last_click == [-1, -1]:
            self.last_click = [event.x-1,event.y-1]
            self.__tmp_objects.append(self.draw_circle([event.x-1,event.y-1],4,"red") )
            self.predict_next_location()
        else:
            # Determine if valid
            if ( self.last_click[0] < self.obj_width and
                 event.x-1 < self.obj_width ):
                print "Improper match!"
            elif( self.last_click[0] >= self.obj_width and
                     event.x-1 >= self.obj_width ):
                print "Improper match!"
            else:
                self.add_measurement(self.last_click,[event.x-1,event.y-1])
            self.last_click = [-1, -1]
            for i in self.__tmp_objects:
                self.canvas.delete(i)
            if (len(self.measurement1) > 3):
                self.transform = solve_affine(self.measurement1,
                                         self.measurement2)
                front = array([[self.image1.scale,0,0],[0,self.image1.scale,0],[0,0,1]])
                back = array([[1/self.image2.scale,0,0],[0,1/self.image2.scale,0],[0,0,1]])
                self.transform = dot(back,dot(self.transform,front))
                mytext = "Transform: "+str(self.transform)
                self.__tmp_objects.append(self.canvas.create_text([self.obj_width/2,50],text=mytext,fill="green",width=self.obj_width))


def main():
    if not sys.argv[1:]:
        print "Usage: python smart_picker.py filename filename"

    smart_picker = App()
    smart_picker.root.mainloop()

if __name__ == "__main__":
    sys.exit(main())
