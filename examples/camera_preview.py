import cv2
import serial
import time

def main():
    print("Opening camera stream for index 2...")
    print("Press 'q' on your keyboard while the window is focused to quit.")
    
    # We use index 2 now because the camera reconnected!
    camera_index = 2
    
    cap = cv2.VideoCapture(camera_index)
    
    if not cap.isOpened():
        print(f"Error: Cannot open camera at index {camera_index}. Try changing the index to 0 or 2.")
        return
    
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        if not ret:
            print("Can't receive frame. Exiting ...")
            break
            
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
