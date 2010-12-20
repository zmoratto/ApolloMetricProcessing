#!/opt/local/bin/python2.6

from Tkinter import *
import Image, ImageTk, sys, os
from numpy import *

from FittingFunction import *
from InterestIO import read_match_file, write_match_file, ip

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
        self.__picked_objects = []
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
            ip1, ip2 = read_match_file( match )
            for i in ip1:
                self.loaded_measurement1.append(i*self.image1.scale)
            for i in ip2:
                self.loaded_measurement2.append(i*self.image2.scale)
            self.draw_loaded_matches()

        self.canvas.pack()

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
        self.__picked_objects.append(self.draw_circle(left,1,"green"))
        self.__picked_objects.append(self.draw_circle(right,1,"green"))
        right[0] = right[0] - self.obj_width
        self.measurement1.append(ip(array(left),5))
        self.measurement2.append(ip(array(right),5))

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
        elif ( measurements == 3 ):
            transform = solve_affine(self.measurement1,
                                     self.measurement2)
        else:
            transform = solve_homography( self.measurement1,
                                          self.measurement2 )

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
            output = sys.argv[1][:sys.argv[1].rfind(".")] + "__" + sys.argv[2][:sys.argv[2].rfind(".")] + ".match"

            # Combined loaded measurements to picked measurements
            ip1 = []
            ip2 = []
            for i in self.measurement1:
                ip1.append(i/self.image1.scale)
            for i in self.loaded_measurement1:
                ip1.append(i/self.image1.scale)
            for i in self.measurement2:
                ip2.append(i/self.image2.scale)
            for i in self.loaded_measurement2:
                ip2.append(i/self.image2.scale)
            write_match_file(output,ip1,ip2)
            for i in self.__picked_objects:
                self.canvas.delete(i)
            for i in self.__match_draw_objects:
                self.canvas.delete(i)
            self.measurement1 = []
            self.measurement2 = []
            self.loaded_measurement1 = []
            self.loaded_measurement2 = []
            ip1, ip2 = read_match_file( output )
            for i in ip1:
                self.loaded_measurement1.append(i*self.image1.scale)
            for i in ip2:
                self.loaded_measurement2.append(i*self.image2.scale)
            self.draw_loaded_matches()

        elif event.char == 'q' or event.char == 'Q':
            print "Quit!"
            sys.exit()
        elif event.char == 'm' or event.char == 'M':
            self.__match_draw_mode = (self.__match_draw_mode + 1 ) % 2
            self.draw_loaded_matches()
        elif event.char == 'r' or event.char == 'R':
            self.loaded_measurement1 = []
            self.loaded_measurement2 = []
            cmd = "ip_guided_match ["+str(self.transform[0][0])+","+str(self.transform[0][1])+","+str(self.transform[0][2])+","+str(self.transform[1][0])+","+str(self.transform[1][1])+","+str(self.transform[1][2])+","+str(self.transform[2][0])+","+str(self.transform[2][1])+","+str(self.transform[2][2])+"] "+sys.argv[1]+" "+sys.argv[2]+" --pass1 100"
            print cmd
            os.system(cmd)
            match = sys.argv[1][:sys.argv[1].rfind(".")] + "__" + sys.argv[2][:sys.argv[2].rfind(".")] + ".match"
            print "Match: ", match
            if ( os.path.exists(match) ):
                ip1, ip2 = read_match_file( match )
                for i in ip1:
                    self.loaded_measurement1.append(i*self.image1.scale)
                for i in ip2:
                    self.loaded_measurement2.append(i*self.image2.scale)
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
                self.transform = solve_homography(self.measurement1,
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
