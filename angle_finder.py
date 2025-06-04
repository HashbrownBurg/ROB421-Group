import cv2
import numpy as np
import time



class angle_finder:
    def __init__(self, camera_port=0, face_cascade='C:\\Users\\galli\\Downloads\\haarcascade_frontalface_alt.xml', hand_cascade='C:\\Users\\galli\\Downloads\\rpalm.xml'):
        self.handCascade = cv2.CascadeClassifier(hand_cascade)
        self.faceCascade = cv2.CascadeClassifier(face_cascade)
        self.video_capture = cv2.VideoCapture(camera_port)
        self.speaker_face_coords = None
        self.speaker_distance = 1e87
        self.last_face = None
        self.last_hand = None
        np.set_printoptions(legacy='1.25') #Useful for debugging

    #Lists the available ports for video use.
    def list_ports(self):
        """
        Test the ports and returns a tuple with the available ports and the ones that are working.
        """
        print("Finding usable ports. This may take awhile.")
        non_working_ports = []
        dev_port = 0
        working_ports = []
        available_ports = []
        while len(non_working_ports) < 6: # if there are more than 5 non working ports stop the testing. 
            camera = cv2.VideoCapture(dev_port)
            if not camera.isOpened():
                non_working_ports.append(dev_port)
                print("Port %s is not working." %dev_port)
            else:
                is_reading, img = camera.read()
                w = camera.get(3)
                h = camera.get(4)
                if is_reading:
                    print("Port %s is working and reads images (%s x %s)" %(dev_port,h,w))
                    working_ports.append(dev_port)
                else:
                    print("Port %s for camera ( %s x %s) is present but does not reads." %(dev_port,h,w))
                    available_ports.append(dev_port)
            dev_port +=1
        return available_ports,working_ports,non_working_ports
    
    #Calculates a mm depth from a pixel distance.
    def depth_from_pix(self, pix):
        '''
        Uses linear interpolation to estimate the number of mm/pixel.
        Based on a 640x480 image.
        May need recalibration with a different image resolution.
        pix: Integer distance in pixels. 
        '''
        mm = round((-2.273*pix+1227), 2)
        return mm

    #Calculates a mm distance from a pixel distance.
    def mm_from_pix(self, pix):
        '''
        Uses linear interpolation to estimate the number of mm/pixel.
        Based on a 640x480 image.
        May need recalibration with a different image resolution.
        pix: Integer distance in pixels. 
        '''
        mm = round((2.273 * pix), 2)
        return mm
    
    #Closes out the camera feed and releases the cameras.
    def end_camera(self):
        self.video_capture.release()
        cv2.destroyAllWindows()
    
    #Calculates the three dimensional distance between two points.
    def threeD_dist(self, loc_one, loc_two):
        '''
        loc_one = (x_coord_mm, y_coord_mm, z_coord_mm)
        loc_two = (x_coord_mm, y_coord_mm, z_coord_mm)
        '''
        x_diff = np.square(loc_two[0] - loc_one[0])
        y_diff = np.square(loc_two[1] - loc_one[1])
        z_diff = np.square(loc_two[2] - loc_one[2])

        dist = round(np.sqrt(x_diff + y_diff + z_diff), 3)
        
        return dist

    #Finds the mm coordinates for each head and returns an array of coords in tuples
    def find_heads(self, frame, gray, origin):
        '''
        frame: an image in which to find heads
        gray: grascale image of the frame in which to find heads
        origin: the designated origin of the image in pixel coordinates. Should be: (x, y)
        '''

        #Detect faces in the frame
        faces = self.faceCascade.detectMultiScale(gray,scaleFactor=1.1,minNeighbors=5,minSize=(30, 30),)

        #Initialize a list of head coordinates
        head_list = []
        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

            #Finds the pixel center location of the face
            face_center_pix = (int(x+w/2), int(y+h/2))
            
            #Estimates the distance of the face from the camera in mm
            depth = self.depth_from_pix(w)
            
            #Calculates the pixel location of the face relative to the center of the camera
            relative_center_pix = (origin[0]-face_center_pix[0], origin[1]-face_center_pix[1])

            #Estimates the location of the face relative to the center of the camera in mm and adds it to the list
            threeD_face_loc = (self.mm_from_pix(relative_center_pix[0]), self.mm_from_pix(relative_center_pix[1]), depth)
            head_list.append(threeD_face_loc)
            
            #Writes stuff on the frame
            loc_text = str(threeD_face_loc) + 'mm'
            cv2.putText(frame, loc_text, (face_center_pix[0], face_center_pix[1]), cv2.FONT_HERSHEY_PLAIN, 1, (127,255,127), 2)

        return head_list
    
    #Finds a person with a raised hand. Returns the angle WRT the camera's center.
    def find_speaker(self):
        #Initialize some variables.
        start = time.time()
        self.last_face = None
        self.last_hand = None
        self.speaker_face_coords = None
        self.speaker_distance = 1e87

        #Go until it finds something.
        while True:
            # Capture frame-by-frame
            ret, frame = self.video_capture.read()

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            #Put a dot in the center of the window
            height, width = frame.shape[:2]
            origin = (int(width/2), int(height/2))  #X, Y
            cv2.circle(frame,(origin[0],origin[1]), 3, (0,0,255), 2)

            hands = self.handCascade.detectMultiScale(gray,scaleFactor=1.1,minNeighbors=5,minSize=(30, 30),)


            head_coord_list = self.find_heads(frame, gray, origin)

            try:
                threeD_hand_loc = None
                x_hand = hands[0][0]
                y_hand = hands[0][1]
                w_hand = hands[0][2]
                h_hand = hands[0][3]

                hand_depth = self.depth_from_pix(w_hand)
                hand_center_pix = (int(x_hand+w_hand/2), int(y_hand+h_hand/2))
                hand_relative_center_pix = (origin[0]-hand_center_pix[0], origin[1]-hand_center_pix[1])

                threeD_hand_loc = (self.mm_from_pix(hand_relative_center_pix[0]), self.mm_from_pix(hand_relative_center_pix[1]), hand_depth)

            except:
                #This triggers if no hands are detected
                pass

            #If at least one face and one hand is detected
            if (len(head_coord_list) >= 1) and (threeD_hand_loc is not None):
                for threeD_face_loc in head_coord_list:
                    self.last_face = threeD_face_loc
                    hand_face_dist = self.threeD_dist(threeD_face_loc, threeD_hand_loc)
                    if hand_face_dist < self.speaker_distance:
                        self.speaker_face_coords = threeD_face_loc
                        self.speaker_distance = hand_face_dist

            #If one face is detected and there is a hand        
            elif (len(head_coord_list) == 1) and (threeD_hand_loc is not None):
                self.speaker_face_coords = head_coord_list[0]

            #If faces are detected but no hands
            elif (len(head_coord_list) >= 1) and (threeD_hand_loc is None):
                self.last_face = head_coord_list[0]

            #If hands are detected but no faces
            elif (len(head_coord_list) == 0) and (threeD_hand_loc is not None):
                self.last_hand = threeD_hand_loc

            # Display the resulting frame, not needed except for debugging :(
            cv2.imshow('Video', frame)

            #Here is where the code is meant to return values if it finds stuff
            if self.speaker_face_coords is not None:
                hori_angle = round((np.arctan(self.speaker_face_coords[0]/self.speaker_face_coords[2]) * 180 / np.pi), 2)
                vert_angle = round((np.arctan(self.speaker_face_coords[1]/self.speaker_face_coords[2]) * 180 / np.pi), 2)
                return (hori_angle, vert_angle)
            
            #If time runs out, return something so it doesn't stall the program
            if ((time.time() - start) > 10):
                #Check one last time that there is no speaker coords
                if self.speaker_face_coords is not None:
                    hori_angle = round((np.arctan(self.speaker_face_coords[0]/self.speaker_face_coords[2]) * 180 / np.pi), 2)
                    vert_angle = round((np.arctan(self.speaker_face_coords[1]/self.speaker_face_coords[2]) * 180 / np.pi), 2)
                    return (hori_angle, vert_angle)
                #Checks if there was a face seen recently and returns those angles
                elif self.last_face is not None:
                    hori_angle = round((np.arctan(self.last_face[0]/self.last_face[2]) * 180 / np.pi), 2)
                    vert_angle = round((np.arctan(self.last_face[1]/self.last_face[2]) * 180 / np.pi), 2)
                    return (hori_angle, vert_angle)
                #If no faces seen, return the angle of the last hand
                elif self.last_hand is not None:
                    hori_angle = round((np.arctan(self.last_hand[0]/self.last_hand[2]) * 180 / np.pi), 2)
                    vert_angle = round((np.arctan(self.last_hand[1]/self.last_hand[2]) * 180 / np.pi), 2)
                    return (hori_angle, vert_angle)
                #If nothing was seen, return None
                else:
                    return None

            #Don't push q or the entire class will need to be restarted
            if cv2.waitKey(1) & 0xFF == ord('q'):
                # When everything is done, release the capture
                self.end_camera()
                return None
            

if __name__ == '__main__':
    face_cascade_file='C:\\Users\\galli\\Downloads\\haarcascade_frontalface_alt.xml'
    hand_cascade_file='C:\\Users\\galli\\Downloads\\rpalm.xml'

    angle_finder = angle_finder(camera_port=0, face_cascade=face_cascade_file, hand_cascade=hand_cascade_file)

    angles = angle_finder.find_speaker()
    print(f'Speaker Angle: {angles}')

    time.sleep(3)

    angles = angle_finder.find_speaker()
    print(f'Speaker Angle: {angles}')

    angle_finder.end_camera()




