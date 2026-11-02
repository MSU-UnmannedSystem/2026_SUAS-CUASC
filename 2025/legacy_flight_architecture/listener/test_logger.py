from legacy_flight_architecture.listener.Logger import TelemetryLogger
import time

print("Initializing Logger...")
logger = TelemetryLogger("test_flight_log.csv")

print("Simulating 3 seconds of flight data...")
for i in range(3):
    # log_data(d_lat, d_lon, d_alt, pitch, roll, yaw, t_lat, t_lon)
    logger.log_data(42.72, -84.48, 50.5 + i, 0.1, 0.2, 90.0, 42.73, -84.49)
    time.sleep(1)

print("Done! Check test_flight_log.csv")
