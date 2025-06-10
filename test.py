import cv2
import time


# Create a VideoCapture object
cap = cv2.VideoCapture('video.mp4')

# Check if video opened successfully
if not cap.isOpened():
    print("Error opening video file")
    exit()

# Create a named window
cv2.namedWindow('rtabmap', cv2.WINDOW_AUTOSIZE)  # WINDOW_AUTOSIZE prevents resize controls
cv2.setWindowProperty('rtabmap', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)  # Optional: make fullscreen

time.sleep(10)


# Read until video is completed
while cap.isOpened():
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    if ret:
        # Display the resulting frame
        cv2.imshow('rtabmap', frame)
        
        # Press Q on keyboard or ESC to exit
        key = cv2.waitKey(25)
        if key & 0xFF in (ord('q'), 27):  # 27 is ESC key
            break
    else:
        break

# Release the video capture object and close windows
cap.release()
cv2.destroyAllWindows()