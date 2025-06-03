

import cv2

# haar cascade detection modules to use in the face detection. AKA what features to look for.
face_classifier = cv2.CascadeClassifier("ROB421-Group/haar_cascades/haarcascade_frontalface_default.xml") 
hand_classifier = cv2.CascadeClassifier('ROB421-Group/haar_cascades/rpalm.xml') 


video_capture = cv2.VideoCapture(0) # take video from main camera 

# find the closest person to the raised hand. 
# If multiple hands, defaults to hand with smallest x value (speaker's right). 
# If no hands, defaults to face with smallest x value
def detect_bounding_box(vid):
    gray_image = cv2.cvtColor(vid, cv2.COLOR_BGR2GRAY) #convert image to greyscale
    faces = face_classifier.detectMultiScale(gray_image, 1.1, 5, minSize=(40, 40))  # use the face detection module to find all the faces
    hand = hand_classifier.detectMultiScale(gray_image, 1.1, 4, minSize=(40, 40))   # use the hand detection module to find all the hands
    
    
    #Find the face closest to the hand (assumes only one hand is found - if multiple, uses hand with smallest x (speaker's right))
    try:
        speaker_face = faces[0] # default to face on audience's right

        # If there is a hand found, find the face that is closest 
        for face_index in faces:
            try:
                if abs(face_index[0]-hand[0][0]) < abs(speaker_face[0]-hand[0][0]):
                    speaker_face = face_index
            except:
                print("no hand detected")

        #Draw a Purple box around hands for debugging purposes
        for (x, y, w, h) in hand: 
            cv2.rectangle(vid, (x, y), (x + w, y + h), (100, 0, 100), 4)

        #Create a Green rectangle around each face
        for (x, y, w, h) in faces: 
            cv2.rectangle(vid, (x, y), (x + w, y + h), (0, 255, 0), 4)
        

        #Draw a blue box around the person with their hand raised
        cv2.rectangle(vid, (speaker_face[0], speaker_face[1]), (speaker_face[0] + speaker_face[2], speaker_face[1] + speaker_face[3]), (255, 0, 0), 4)

        return speaker_face
    except:
        print("No faces found")
        return None


    

while True:

    result, video_frame = video_capture.read()  # read frames from the video
    if result is False:
        break  # terminate the loop if the frame is not read successfully

    face = detect_bounding_box(video_frame)  # this is the face of the speaker (hopefully)

    cv2.imshow(
        "My Face Detection Project", video_frame
    )  # display the processed frame in a window named "My Face Detection Project"

    #press 'q' to stop video
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

video_capture.release()
cv2.destroyAllWindows()

