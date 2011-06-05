#!/usr/bin/env python2.7

from Tkinter import *
import Image, ImageTk, sys, os
from numpy import *

# Hack to use my modules
sys.path.append( os.path.realpath(__file__)[:-19]+"libexec" )

from FittingFunction import *
from InterestIO import read_match_file, write_match_file, ip

class ReducedImage:
    def __init__(self, photoimage, scale):
        self.photoimage = photoimage
        self.scale = scale

class App:
    def __init__(self, image_name1, image_name2 ):
        self.root = Tk()
        self.root.resizable(width=True,height=True)
        self.obj_height = self.root.winfo_screenheight()
        #self.obj_width  = self.root.winfo_screenwidth()/2*0.8
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
        self.transform = identity(3,float)
        self.image_name1 = image_name1
        self.image_name2 = image_name2
        self.match_file = image_name1[:image_name1.rfind(".")] + "__" + image_name2[:image_name2.rfind(".")] + ".match"
        print "Width height: %i %i\n" % (self.obj_width, self.obj_height)

        # Loading up input images
        self.image1 = self.load_image( image_name1 )
        self.image2 = self.load_image( image_name2 )

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
        print "Match: ", self.match_file
        if ( os.path.exists(self.match_file) ):
            ip1, ip2 = read_match_file( self.match_file )
            for i in ip1:
                self.loaded_measurement1.append(i*self.image1.scale)
            for i in ip2:
                self.loaded_measurement2.append(i*self.image2.scale)
            self.draw_loaded_matches()
            self.find_homography()
            self.update_transform()
        self.canvas.pack()
        if ( not os.path.exists(self.match_file) ):
            self.predict_homography()

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
        self.obj_height = int(im_scale*im_h)
        im = im.resize([int(im_scale*im_w),int(im_scale*im_h)])
        imtk = ImageTk.PhotoImage( im, palette=256 )
        return ReducedImage(imtk, im_scale)

    def draw_circle( self, center, radius, color ):
        return self.canvas.create_oval(center[0]-radius,center[1]-radius,
                                       center[0]+radius,center[1]+radius,
                                       outline=color)

    def draw_line( self, p1, p2, color ):
        return self.canvas.create_line(p1[0],p1[1],p2[0],p2[1],fill=color)

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

    def update_transform( self ):
        if ( len(self.loaded_measurement1) > 3 ):
            # Mix our measurements
            m1 = self.measurement1 + self.loaded_measurement1
            m2 = self.measurement2 + self.loaded_measurement2
            self.transform = solve_homography( m1, m2 )
            self.display_transform()
            return
        measurements = len(self.measurement1)
        self.transform = identity(3,float)
        if ( measurements == 0 ):
            # Can't do anythin
            return
        elif ( measurements == 1 ):
            self.transform[0][2] = self.measurement2[0][0] - self.measurement1[0][0]
            self.transform[1][2] = self.measurement2[0][1] - self.measurement1[0][1]
        elif ( measurements == 2 ):
            self.transform = solve_euclidean(self.measurement1,
                                             self.measurement2)
        elif ( measurements == 3 ):
            self.transform = solve_affine(self.measurement1,
                                          self.measurement2)
        else:
            self.transform = solve_homography( self.measurement1,
                                               self.measurement2 )
        self.display_transform()

    def display_transform(self):
        # Write out the current transform / replace with black rectangle
        # mytext = "Transform: "+str(self.transform)
        # self.__tmp_objects.append(self.canvas.create_text([self.obj_width/2,50],text=mytext,fill="green",width=self.obj_width))

        # Draw forward bbox
        right_bbox = []
        prediction = dot(self.transform,array([0,0,1]))
        prediction = prediction/prediction[2]
        right_bbox.append( array([prediction[0],prediction[1]]) )
        prediction = dot(self.transform,array([self.obj_width,0,1]))
        prediction = prediction/prediction[2]
        right_bbox.append( array([prediction[0],prediction[1]]) )
        prediction = dot(self.transform,array([self.obj_width,self.obj_height,1]))
        prediction = prediction/prediction[2]
        right_bbox.append( array([prediction[0],prediction[1]]) )
        prediction = dot(self.transform,array([0,self.obj_height,1]))
        prediction = prediction/prediction[2]
        right_bbox.append( array([prediction[0],prediction[1]]) )

        trans = array([self.obj_width,0])
        self.__tmp_objects.append(self.draw_line(right_bbox[0]+trans,right_bbox[1]+trans,"green"));
        self.__tmp_objects.append(self.draw_line(right_bbox[1]+trans,right_bbox[2]+trans,"green"));
        self.__tmp_objects.append(self.draw_line(right_bbox[2]+trans,right_bbox[3]+trans,"green"));
        self.__tmp_objects.append(self.draw_line(right_bbox[3]+trans,right_bbox[0]+trans,"green"));

        left_bbox = []
        itransform = linalg.pinv( self.transform )
        prediction = dot(itransform,array([0,0,1]))
        prediction = prediction/prediction[2]
        left_bbox.append( array([prediction[0],prediction[1]]) )
        prediction = dot(itransform,array([self.obj_width,0,1]))
        prediction = prediction/prediction[2]
        left_bbox.append( array([prediction[0],prediction[1]]) )
        prediction = dot(itransform,array([self.obj_width,self.obj_height,1]))
        prediction = prediction/prediction[2]
        left_bbox.append( array([prediction[0],prediction[1]]) )
        prediction = dot(itransform,array([0,self.obj_height,1]))
        prediction = prediction/prediction[2]
        left_bbox.append( array([prediction[0],prediction[1]]) )
        self.__tmp_objects.append(self.draw_line(left_bbox[0],left_bbox[1],"blue"));
        self.__tmp_objects.append(self.draw_line(left_bbox[1],left_bbox[2],"blue"));
        self.__tmp_objects.append(self.draw_line(left_bbox[2],left_bbox[3],"blue"));
        self.__tmp_objects.append(self.draw_line(left_bbox[3],left_bbox[0],"blue"));

    def predict_next_location(self):
        measurements = len(self.measurement1)
        if (measurements == 0 and len(self.loaded_measurement1) < 4):
            return
        prediction = array([0,0])
        if ( self.last_click[0] >= self.obj_width ):
            # Run backwards
            itransform = linalg.pinv( self.transform )
            prediction = dot(itransform,array([self.last_click[0]-self.obj_width,self.last_click[1],1]));
            prediction = prediction/prediction[2]
        else:
            # Run forwards
            prediction = dot(self.transform,array([self.last_click[0],self.last_click[1],1]));
            prediction = prediction/prediction[2]
            prediction[0] = prediction[0] + self.obj_width
        prediction = array([prediction[0],prediction[1]])
        if ( measurements < 4 ):
            self.__tmp_objects.append(self.draw_circle(prediction,100-measurements*25,"yellow"))
        else:
            self.__tmp_objects.append(self.draw_circle(prediction,20,"yellow"))

    def save_measurements( self ):
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
        write_match_file(self.match_file,ip1,ip2)
        for i in self.__picked_objects:
            self.canvas.delete(i)
        for i in self.__match_draw_objects:
            self.canvas.delete(i)
        self.measurement1 = []
        self.measurement2 = []
        self.loaded_measurement1 = []
        self.loaded_measurement2 = []
        ip1, ip2 = read_match_file( self.match_file )
        for i in ip1:
            self.loaded_measurement1.append(i*self.image1.scale)
        for i in ip2:
            self.loaded_measurement2.append(i*self.image2.scale)
        self.draw_loaded_matches()

    def delete_measurements( self ):
        self.loaded_measurement1 = []
        self.loaded_measurement2 = []
        self.draw_loaded_matches()
        os.system("rm "+self.match_file)
        self.update_transform()

    def reload_measurements( self ):
        self.loaded_measurement1 = []
        self.loaded_measurement2 = []
        if ( os.path.exists(self.match_file) ):
            ip1, ip2 = read_match_file( self.match_file )
            for i in ip1:
                self.loaded_measurement1.append(i*self.image1.scale)
            for i in ip2:
                self.loaded_measurement2.append(i*self.image2.scale)
            self.draw_loaded_matches()

    def transform_search_measurements( self ):
        # Copy and scale transform to real measurements
        front = array([[self.image1.scale,0,0],[0,self.image1.scale,0],[0,0,1]])
        back = array([[1/self.image2.scale,0,0],[0,1/self.image2.scale,0],[0,0,1]])
        xform = dot(back,dot(self.transform,front))

        print "Using xform: "+str(xform)

        # Run cmd
        cmd_path = os.path.realpath(__file__)[:-19]+"libexec/"
        cmd = "ip_guided_match ["+str(xform[0][0])+","+str(xform[0][1])+","+str(xform[0][2])+","+str(xform[1][0])+","+str(xform[1][1])+","+str(xform[1][2])+","+str(xform[2][0])+","+str(xform[2][1])+","+str(xform[2][2])+"] "+self.image_name1+" "+self.image_name2+" --pass1 100"
        print cmd
        os.system(cmd_path+cmd)
        self.reload_measurements()

    def ba_filter_measurements( self ):
        cmd_path = os.path.realpath(__file__)[:-19]+"libexec/"
        cmd = "ba_filter "+self.image_name1+" "+self.image_name2
        print cmd
        os.system(cmd_path+cmd)
        self.reload_measurements()

    def kriging_search_measurements( self ):
        cmd_path = os.path.realpath(__file__)[:-19]+"libexec/"
        cmd = "kriging_guided_match -s 5 "+self.image_name1+" "+self.image_name2
        print cmd
        os.system(cmd_path+cmd)
        self.reload_measurements()

    def equalize_measurements( self ):
        cmd = "reduce_match -n5 -m25 "+self.match_file
        print cmd
        os.system(cmd)
        self.reload_measurements()

    def find_homography( self ):
        # Forces a solve
        self.save_measurements()
        cmd_path = os.path.realpath(__file__)[:-19]+"libexec/"
        p = subprocess.Popen(cmd_path+"homography_fit "+self.match_file,
                             shell=True,stdout=subprocess.PIPE);
        cmd_return = p.stdout.readline().strip()
        text = cmd_return[cmd_return.find(":")+13:].strip("((").strip("))").replace(")(",",").split(",")
        solution = identity(3,float)
        solution[0,0:3] = [float(text[0]), float(text[1]), float(text[2])]
        solution[1,0:3] = [float(text[3]), float(text[4]), float(text[5])]
        solution[2,0:3] = [float(text[6]), float(text[7]), float(text[8])]
        print "Found: "+str(solution)
        # scale transform to screen
        front = array([[1/self.image1.scale,0,0],[0,1/self.image1.scale,0],[0,0,1]])
        back = array([[self.image2.scale,0,0],[0,self.image2.scale,0],[0,0,1]])
        self.transform = dot(back,dot(solution,front))
        self.display_transform()

    def predict_homography( self ):
        cmd_path = os.path.realpath(__file__)[:-19]+"libexec/"
        cmd = cmd_path+"predict_homography "+self.image_name1+" "+self.image_name2
        p = subprocess.Popen( cmd, shell=True, stdout=subprocess.PIPE );
        p.stdout.readline()
        p.stdout.readline()
        cmd_return = p.stdout.readline().strip()
        text = cmd_return[cmd_return.find(":")+13:].strip("((").strip("))").replace(")(",",").split(",")
        solution = identity(3,float)
        try:
            solution[0,0:3] = [float(text[0]), float(text[1]), float(text[2])]
            solution[1,0:3] = [float(text[3]), float(text[4]), float(text[5])]
            solution[2,0:3] = [float(text[6]), float(text[7]), float(text[8])]
        except ValueError:
            print "Unable to predict homography.\n"
            return
        print "Found: "+str(solution)
        # scale transform to screen
        front = array([[1/self.image1.scale,0,0],[0,1/self.image1.scale,0],[0,0,1]])
        back = array([[self.image2.scale,0,0],[0,self.image2.scale,0],[0,0,1]])
        self.transform = dot(back,dot(solution,front))
        self.display_transform()

    def key_press(self, event):
        if event.char == 's' or event.char == 'S':
            self.save_measurements()
        elif event.char == 'q' or event.char == 'Q':
            print "Quit!"
            sys.exit()
        elif event.char == 'm' or event.char == 'M':
            self.__match_draw_mode = (self.__match_draw_mode + 1 ) % 2
            self.draw_loaded_matches()
        elif event.char == 'd' or event.char == 'D':
            self.delete_measurements()
        elif event.char == 'r' or event.char == 'R':
            self.transform_search_measurements()
        elif event.char == 'f' or event.char == 'F':
            self.ba_filter_measurements()
        elif event.char == 'k' or event.char == 'K':
            self.kriging_search_measurements()
        elif event.char == 'e' or event.char == 'E':
            self.equalize_measurements()
        elif event.char == 'p' or event.char == 'P':
            self.predict_homography()
        elif event.char == 'h' or event.char == 'H':
            self.find_homography()

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
            self.update_transform()

def main():
    if not sys.argv[1:]:
        print "Usage: python smart_picker.py filename filename"
        print " or  : python smart_picker.py match_file_name (if input tif)"

    if len(sys.argv) == 2:
        image1 = sys.argv[1].split("__")[0] + ".tif"
        image2 = sys.argv[1].split("__")[1][:-5] + "tif"
        smart_picker = App(image1,image2)
        smart_picker.root.mainloop()
    elif len(sys.argv) == 3:
        smart_picker = App(sys.argv[1],sys.argv[2])
        smart_picker.root.mainloop()

if __name__ == "__main__":
    sys.exit(main())
