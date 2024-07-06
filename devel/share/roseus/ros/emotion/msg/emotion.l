;; Auto-generated. Do not edit!


(when (boundp 'emotion::emotion)
  (if (not (find-package "EMOTION"))
    (make-package "EMOTION"))
  (shadow 'emotion (find-package "EMOTION")))
(unless (find-package "EMOTION::EMOTION")
  (make-package "EMOTION::EMOTION"))

(in-package "ROS")
;;//! \htmlinclude emotion.msg.html


(defclass emotion::emotion
  :super ros::object
  :slots (_dominant_emotion _face_found _model_confidence ))

(defmethod emotion::emotion
  (:init
   (&key
    ((:dominant_emotion __dominant_emotion) "")
    ((:face_found __face_found) nil)
    ((:model_confidence __model_confidence) 0.0)
    )
   (send-super :init)
   (setq _dominant_emotion (string __dominant_emotion))
   (setq _face_found __face_found)
   (setq _model_confidence (float __model_confidence))
   self)
  (:dominant_emotion
   (&optional __dominant_emotion)
   (if __dominant_emotion (setq _dominant_emotion __dominant_emotion)) _dominant_emotion)
  (:face_found
   (&optional (__face_found :null))
   (if (not (eq __face_found :null)) (setq _face_found __face_found)) _face_found)
  (:model_confidence
   (&optional __model_confidence)
   (if __model_confidence (setq _model_confidence __model_confidence)) _model_confidence)
  (:serialization-length
   ()
   (+
    ;; string _dominant_emotion
    4 (length _dominant_emotion)
    ;; bool _face_found
    1
    ;; float64 _model_confidence
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _dominant_emotion
       (write-long (length _dominant_emotion) s) (princ _dominant_emotion s)
     ;; bool _face_found
       (if _face_found (write-byte -1 s) (write-byte 0 s))
     ;; float64 _model_confidence
       (sys::poke _model_confidence (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _dominant_emotion
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _dominant_emotion (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _face_found
     (setq _face_found (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _model_confidence
     (setq _model_confidence (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get emotion::emotion :md5sum-) "8f1330b96c69645b621bb929fd2da385")
(setf (get emotion::emotion :datatype-) "emotion/emotion")
(setf (get emotion::emotion :definition-)
      "string dominant_emotion
bool face_found
float64 model_confidence

")



(provide :emotion/emotion "8f1330b96c69645b621bb929fd2da385")


