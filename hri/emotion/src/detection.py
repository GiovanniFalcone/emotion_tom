import cv2
import time
import numpy as np
import mediapipe as mp

from deepface import DeepFace

class Detection:

    def __init__(self):
        # Configure Face Detection Model
        # - model_selection=0 ->to detect faces within the range of 2 meters from the camera
        # - min_detection_confidence -> Minimum confidence value ([0.0, 1.0]) from the face detection model for the detection to be considered successful. Default to 0.5.
        mp_face_detection = mp.solutions.face_detection
        self.face_detector = mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.6)
        # Configure face mesh for head pose estimation
        mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = mp_face_mesh.FaceMesh(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        # in order to draw rectangle around the face - also landmarks, etc...
        # mp_drawing = mp.solutions.drawing_utils
        # drawing_spec = mp_drawing.DrawingSpec(color=(128, 0, 128), thickness=2, circle_radius=1)
    
    @staticmethod
    # credit: https://medium.com/@vabhinav991222/effortless-face-detection-and-distance-measurement-enhancing-interaction-with-computer-vision-d96e280d16a0
    def measure_distance(face_width_pixels):
        focal_length = 600      # Set the focal length based on your camera specifications
        avg_face_width = 14     # Set the average width of a face in centimeters
        return (avg_face_width * focal_length) / face_width_pixels  

    @staticmethod
    def analyze_emotion(face_detected, face_coordinates, max_distance):
        _, _, w, _ = face_coordinates
        # Measure the distance to the detected face
        distance = Detection.measure_distance(w)
        if distance > max_distance:
            print(f"Face distance greater than 100cm. Distance is {distance:.2f} ...")
            return None
        else:
            # now analyze face_roi for emotion recognition
            # start = time.time()
            result =  DeepFace.analyze(face_detected, actions=['emotion'], enforce_detection=False)
            # end = time.time() - start
            # print(f"Time taken for DeepFace analysis: {end:.4f} seconds -> {result[0]['dominant_emotion']}")
            return result

    def face_detection_mediapipe(self, frame):
        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        frame.flags.writeable = False

        # OpenCV format is BGR(Blue, Green, Red) while mediapipe accepts RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # process rgb frame and returns a list of faces detected
        results = self.face_detector.process(rgb_frame)
        
        # once that mediapipe has analyzed the frame, it can be set to true again
        frame.flags.writeable = True
        
        # checking if face is present in detection or not
        if not results.detections:
            return None, None, None
        else:
            # list of faces and their informations
            face_boxes = []
            # for each face get info
            for face in results.detections:
                # get rectangle around the face detected 
                box_face = face.location_data.relative_bounding_box
                # get frame size
                ih, iw, _ = frame.shape
                # coordinates are [0, 1] value -> multiply for frame size to get absolute values
                x = int(box_face.xmin * iw)
                y = int(box_face.ymin * ih)
                w = int(box_face.width * iw)
                h = int(box_face.height * ih)

                # Check if box is inside the frame
                if x >= 0 and y >= 0 and x + w <= iw and y + h <= ih:
                    # area face
                    a = w * h
                    # append face info into the list
                    face_boxes.append((x, y, w, h, a))
            
            # idea: sort all feces detected, get the largest one (i.e the closest) and analyze it (other faces will not be analyzed)
            if face_boxes:
                # sorts face areas from largest to smallest 
                face_boxes.sort(key=lambda x: x[4], reverse=True)
                # get the largest one (i.e the closest)
                x, y, w, h, _ = face_boxes[0]

                # extract ROI (region of interest) in order to analyze less data for emotion recognition (deepface)
                face_roi = frame[y:y + h, x:x + w]
                # resize ROI to get a faster analysis
                small_face_roi = cv2.resize(face_roi, (112, 112))       
                # face with padding to get spacial context (for head pose)
                padding = 100 
                x1 = max(x - padding, 0)
                y1 = max(y - padding, 0)
                x2 = min(x + w + padding, frame.shape[1])
                y2 = min(y + h + padding, frame.shape[0])
                face_with_context = frame[y1:y2, x1:x2]       

                return small_face_roi, face_with_context, (x, y, w, h)
            else:
                return None, None, None
        
    # credit: https://medium.com/@jaykumaran2217/real-time-head-pose-estimation-facemesh-with-mediapipe-and-opencv-a-comprehensive-guide-b63a2f40b7c6
    def head_pose(self, mirrored_frame, image):
        image.flags.writeable = False

        image = cv2.cvtColor(cv2.flip(image,1),cv2.COLOR_BGR2RGB) # flipped for selfie view

        results = self.face_mesh.process(image)

        image.flags.writeable = True

        image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)

        img_h , img_w, img_c = image.shape
        face_2d = []
        face_3d = []

        if results.multi_face_landmarks:
            for face_landmarks in results.multi_face_landmarks:
                for idx, lm in enumerate(face_landmarks.landmark):
                    if idx in [33, 263, 1, 61, 291, 199]:
                        x, y = int(lm.x * img_w), int(lm.y * img_h)

                        face_2d.append([x,y])
                        face_3d.append(([x,y,lm.z]))


                #Get 2d Coord
                face_2d = np.array(face_2d,dtype=np.float64)

                face_3d = np.array(face_3d,dtype=np.float64)

                focal_length = 1 * img_w

                cam_matrix = np.array([[focal_length, 0, img_h/2],
                                    [0, focal_length, img_w/2],
                                    [0, 0, 1]])
                distortion_matrix = np.zeros((4,1), dtype=np.float64)

                success, rotation_vec, translation_vec = cv2.solvePnP(face_3d, face_2d, cam_matrix, distortion_matrix)

                #getting rotational of face
                rmat, jac = cv2.Rodrigues(rotation_vec)

                angles, mtxR, mtxQ, Qx, Qy, Qz = cv2.RQDecomp3x3(rmat)

                x = angles[0] * 360
                y = angles[1] * 360
                z = angles[2] * 360

                #here based on axis rot angle is calculated
                if y < -15:     text = "Looking Left"
                elif y > 15:    text = "Looking Right"
                elif x < -15:   text = "Looking Down"
                elif x > 15:    text = "Looking Up"
                else:           text = "Forward"

                if text is not None:
                    # Display the head pose (forward, looking right, ...) and coordinates
                    cv2.putText(mirrored_frame, text, (200, 30), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 255, 255), 2, cv2.LINE_AA)
                    cv2.putText(mirrored_frame, "x: " + str(np.round(x, 2)), (mirrored_frame.shape[1] - 100, 30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255), 2, cv2.LINE_AA)
                    cv2.putText(mirrored_frame, "y: " + str(np.round(y, 2)), (mirrored_frame.shape[1] - 100, 45), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255), 2, cv2.LINE_AA)
                    cv2.putText(mirrored_frame, "z: " + str(np.round(z, 2)), (mirrored_frame.shape[1] - 100, 60), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255),  2, cv2.LINE_AA)


                return text, (x, y, z)
        else:
            return None, None
