import cv2

def main():
    # Initialize video capture object (0 for default webcam)
    cap = cv2.VideoCapture(0)

    # Check if the webcam was opened successfully
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        exit()

    while True:
        # Read a frame from the webcam
        ret, frame = cap.read()

        # If frame reading failed, break the loop
        if not ret:
            print("Failed to grab frame.")
            break

        # Display the captured frame
        cv2.imshow('Webcam Stream', frame)

        # Exit if 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the VideoCapture object and destroy all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

