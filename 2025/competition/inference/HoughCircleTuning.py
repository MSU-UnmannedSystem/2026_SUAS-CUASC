#!/usr/bin/python
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import time

# This isn't for the detection, it's just to help you find the lower and upper hsv bounds.
# Green represents a contour that has been found, blue represents the biggest contour. Try to get it so that they
# only appear around notes.

MIN_CIRCLES_FOR_BULLSEYE = 3
CENTER_TOLERANCE_PX = 8


def detect_bullseyes(mask: np.ndarray) -> list:
	"""
	Find concentric circles (bullseyes) inside the binary mask using HoughCircles.
	Returns a list of tuples (center, radius, radii_list).
	"""
	blurred = cv2.GaussianBlur(mask, (9, 9), 2)
	circles = cv2.HoughCircles(
		blurred,
		cv2.HOUGH_GRADIENT,
		dp=1.2,
		minDist=30,
		param1=100,
		param2=25,
		minRadius=10,
		maxRadius=0
	)

	results = []
	if circles is None:
		return results

	circles = np.round(circles[0]).astype(int)
	groups = []
	for (x, y, r) in circles:
		assigned = False
		for group in groups:
			gx, gy = group['center']
			if abs(x - gx) <= CENTER_TOLERANCE_PX and abs(y - gy) <= CENTER_TOLERANCE_PX:
				count = len(group['radii'])
				group['center'] = (
					(gx * count + x) / (count + 1),
					(gy * count + y) / (count + 1)
				)
				group['radii'].append(r)
				assigned = True
				break
		if not assigned:
			groups.append({'center': (float(x), float(y)), 'radii': [r]})

	for group in groups:
		if len(group['radii']) >= MIN_CIRCLES_FOR_BULLSEYE:
			cx = int(round(group['center'][0]))
			cy = int(round(group['center'][1]))
			results.append(((cx, cy), max(group['radii']), sorted(group['radii'])))

	return results


def main():
	# Create a window to adjust the lower and upper bounds
	cv2.namedWindow('Trackbars', cv2.WINDOW_NORMAL)  # Use WINDOW_NORMAL to allow resizing
	cv2.resizeWindow('Trackbars', 600, 300)  # Set the size of the window (width, height)

	# Create trackbars
	cv2.createTrackbar('Hue Lower', 'Trackbars', 1, 360, lambda x: None)
	cv2.createTrackbar('Saturation Lower', 'Trackbars', 100, 0xFF, lambda x: None)
	cv2.createTrackbar('Value Lower', 'Trackbars', 100, 0xFF, lambda x: None)
	cv2.createTrackbar('Hue Upper', 'Trackbars', 6, 360, lambda x: None)
	cv2.createTrackbar('Saturation Upper', 'Trackbars', 0xFF, 0xFF, lambda x: None)
	cv2.createTrackbar('Value Upper', 'Trackbars', 0xFF, 0xFF, lambda x: None)

	# Specify the camera index (usually 0 for built-in webcam)
	camera_index = 0

	# Open the camera
	cap = cv2.VideoCapture(camera_index)
	prev_time = time.perf_counter()
	fps = 0.0

	while True:
		try:
			# Capture frame-by-frame
			ret, frame = cap.read()

			if not ret:
				print('Error: Unable to capture frame')
				break

			# Convert frame from BGR to HSV color space
			hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

			# Get current trackbar positions
			hue_lower = cv2.getTrackbarPos('Hue Lower', 'Trackbars')
			saturation_lower = cv2.getTrackbarPos('Saturation Lower', 'Trackbars')
			value_lower = cv2.getTrackbarPos('Value Lower', 'Trackbars')
			hue_upper = cv2.getTrackbarPos('Hue Upper', 'Trackbars')
			saturation_upper = cv2.getTrackbarPos('Saturation Upper', 'Trackbars')
			value_upper = cv2.getTrackbarPos('Value Upper', 'Trackbars')

			# Define lower and upper bounds for orange color in HSV
			lower = np.array([hue_lower, saturation_lower, value_lower])
			upper = np.array([hue_upper, saturation_upper, value_upper])

			# Threshold the HSV image to get only orange colors
			mask = cv2.inRange(hsv, lower, upper)

			# Find contours in the mask
			contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

			# Find the largest contour (clump) of orange pixels
			if contours:
				# Draw everything else it's detecting
				cv2.drawContours(frame, contours, -1, [0xFF, 0, 0], 1)

				# Gets the largest contour and draws it on
				largest_contour = max(contours, key=cv2.contourArea)
				cr = sorted(contours, key=cv2.contourArea)

				try:
					cv2.drawContours(frame, [largest_contour], 0, [0xFF, 0, 0], 2)
					
				except Exception as exc:
					# Not enough contours to draw the additional ones
					print(f'Contour drawing skipped: {exc}')

			# Detect bullseyes inside the current mask
			bullseyes = detect_bullseyes(mask)
			for (center, radius, radii_list) in bullseyes:
				cv2.circle(frame, center, radius, (0, 0xFF, 0), 3)
				cv2.putText(
					frame,
					f'Bullseye ({len(radii_list)})',
					(center[0] - radius, max(center[1] - radius - 10, 20)),
					cv2.FONT_HERSHEY_SIMPLEX,
					0.6,
					(0, 0xFF, 0),
					2,
					cv2.LINE_AA
				)

			# Update FPS measurement
			now = time.perf_counter()
			elapsed = now - prev_time
			if elapsed > 0:
				fps = 1.0 / elapsed
			prev_time = now

			# Overlay FPS
			cv2.putText(
				frame,
				f'FPS: {fps:.1f}',
				(10, 25),
				cv2.FONT_HERSHEY_SIMPLEX,
				0.7,
				(0xFF, 0xFF, 0xFF),
				2,
				cv2.LINE_AA
			)

			# Display the resulting frame
			cv2.imshow('Frame', frame)

			# Break the loop if "q" is pressed
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
		except Exception as exc:
			print(f'Processing error, continuing: {exc}')
			continue

	# Release the capture
	cap.release()
	cv2.destroyAllWindows()


if __name__ == '__main__':
	main()
