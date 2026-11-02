from competition.jetson_rescue.camera import VisionPipeline
import time

def run_test():
    print("Initializing Vision Pipeline...")
    # Make sure camera_index matches your USB port (usually 0 or 1)
    vision = VisionPipeline(camera_index=0) 

    print("Camera active. Show me the red bullseye...")
    
    try:
        while True:
            pixel_x, pixel_y = vision.get_target_pixel()
            
            if pixel_x is not None:
                print(f"[LOCKED] Target Acquired at Pixel: ({pixel_x}, {pixel_y})")
            else:
                print("[SEARCHING] No target detected...")
                
            time.sleep(0.5) # Run twice a second to avoid terminal spam
            
    except KeyboardInterrupt:
        print("\nTest terminated by user.")
        vision.release()

if __name__ == "__main__":
    run_test()
