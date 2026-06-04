import cv2
import serial
import time

def main():
    print("Opening camera stream for index 2...")
    print("Press 'q' on your keyboard while the window is focused to quit.")
    
    # We use index 2 now because the camera reconnected!
    camera_index = 2
    
    cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
    
    while True:
        if not cap.isOpened():
            cap.release()
            cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
            
        if cap.isOpened():
            # Capture frame-by-frame
            ret, frame = cap.read()
            
            if not ret:
                cap.release()
                continue
                
            # Display the resulting frame in a window
            cv2.imshow(f'Andrew Robot Camera (Index {camera_index})', frame)
            
        # Press 'q' to quit
        if cv2.waitKey(1) == ord('q'):
            break
            
    # When everything is done, release the capture and close the window
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
