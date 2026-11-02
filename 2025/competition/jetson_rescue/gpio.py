# export JETSON_MODEL_NAME="JETSON_ORIN_NANO"

import Jetson.GPIO as GPIO
import time

# Pin Definitions
led_pin = 29  # Example: Using physical pin 7 (BOARD mode)

# Pin Setup
GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering
GPIO.setup(led_pin, GPIO.OUT, initial=GPIO.LOW)  # Set pin as an output, initially LOW

try:
    while True:
        GPIO.output(led_pin, GPIO.HIGH)  # Turn LED on
        time.sleep(2)  # Wait for 500ms
        GPIO.output(led_pin, GPIO.LOW)   # Turn LED off
        time.sleep(2)  # Wait for 500ms

except KeyboardInterrupt:
    print("Exiting gracefully")
finally:
    GPIO.cleanup()  # Clean up all GPIOs to release resources

