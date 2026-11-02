import cv2
import numpy as np
import time


WINDOW_TRACKBARS = 'settings'
WINDOW_FEED = 'feed'
WINDOW_RED_GRAY = 'red-gray'
MIN_CONTOUR_AREA = 400
CENTER_TOLERANCE_PX = 12


def detect_red_circles(red_gray: np.ndarray) -> list[tuple[int, int, int]]:
	"""Detect circles on the grayscale representation of red pixels."""
	blurred = cv2.GaussianBlur(red_gray, (9, 9), 2)
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
	if circles is None:
		return []
	return np.round(circles[0]).astype(int).tolist()


def create_trackbars():
	cv2.namedWindow(WINDOW_TRACKBARS, cv2.WINDOW_NORMAL)
	cv2.resizeWindow(WINDOW_TRACKBARS, 600, 300)
	cv2.createTrackbar('Hue Lower 1', WINDOW_TRACKBARS, 0, 179, lambda _: None)
	cv2.createTrackbar('Hue Upper 1', WINDOW_TRACKBARS, 10, 179, lambda _: None)
	cv2.createTrackbar('Hue Lower 2', WINDOW_TRACKBARS, 170, 179, lambda _: None)
	cv2.createTrackbar('Hue Upper 2', WINDOW_TRACKBARS, 179, 179, lambda _: None)
	cv2.createTrackbar('Sat Lower', WINDOW_TRACKBARS, 100, 0xFF, lambda _: None)
	cv2.createTrackbar('Val Lower', WINDOW_TRACKBARS, 100, 0xFF, lambda _: None)
	cv2.createTrackbar('Sat Upper', WINDOW_TRACKBARS, 0xFF, 0xFF, lambda _: None)
	cv2.createTrackbar('Val Upper', WINDOW_TRACKBARS, 0xFF, 0xFF, lambda _: None)


def get_trackbar(name: str) -> int:
	return cv2.getTrackbarPos(name, WINDOW_TRACKBARS)


def main():
	create_trackbars()
	cap = cv2.VideoCapture(0)
	prev_time = time.perf_counter()
	fps = 0.0

	while True:
		try:
			ret, frame = cap.read()
			if not ret:
				print('Warning: unable to grab frame.')
				continue

			hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

			hl1 = get_trackbar('Hue Lower 1')
			hu1 = get_trackbar('Hue Upper 1')
			hl2 = get_trackbar('Hue Lower 2')
			hu2 = get_trackbar('Hue Upper 2')
			sl = get_trackbar('Sat Lower')
			vl = get_trackbar('Val Lower')
			su = get_trackbar('Sat Upper')
			vu = get_trackbar('Val Upper')

			lower1 = np.array([hl1, sl, vl])
			upper1 = np.array([hu1, su, vu])
			lower2 = np.array([hl2, sl, vl])
			upper2 = np.array([hu2, su, vu])

			mask1 = cv2.inRange(hsv, lower1, upper1)
			mask2 = cv2.inRange(hsv, lower2, upper2)
			mask = cv2.bitwise_or(mask1, mask2)

			gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
			red_gray = cv2.bitwise_and(gray, gray, mask=mask)
			_, binary = cv2.threshold(red_gray, 0, 0xFF, cv2.THRESH_BINARY)

			contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
			contour_circles = []
			for contour in contours:
				if cv2.contourArea(contour) < MIN_CONTOUR_AREA:
					continue
				cv2.drawContours(frame, [contour], -1, (0, 0xFF, 0), 2)
				(x, y), radius = cv2.minEnclosingCircle(contour)
				contour_circles.append((int(x), int(y), int(radius)))

			detected_circles = detect_red_circles(red_gray)
			detected_circles = sorted(detected_circles, key=lambda c: c[2], reverse=True)
			filtered = []
			for circle in detected_circles:
				x, y, r = circle
				if any((x - fx) ** 2 + (y - fy) ** 2 <= CENTER_TOLERANCE_PX ** 2 for fx, fy, _ in filtered):
					continue
				filtered.append(circle)
				if len(filtered) == 3:
					break
			detected_circles = filtered
			# for (x, y, r) in detected_circles:
			# 	cv2.circle(frame, (x, y), r, (0xFF, 0, 0), 2)

			for (x, y, r) in contour_circles:
				cv2.circle(frame, (x, y), 4, (0, 0xFF, 0), -1)
				cv2.putText(
					frame,
					f'({x}, {y})',
					(x + 10, y - 10),
					cv2.FONT_HERSHEY_SIMPLEX,
					0.7,
					(0, 0xFF, 0),
					5,
					cv2.LINE_AA
				)

			now = time.perf_counter()
			elapsed = now - prev_time
			if elapsed > 0:
				fps = 1.0 / elapsed
			prev_time = now

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

			cv2.imshow(WINDOW_FEED, frame)
			cv2.imshow(WINDOW_RED_GRAY, red_gray)

			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
		except Exception as exc:
			print(f'Processing error, continuing: {exc}')
			continue

	cap.release()
	cv2.destroyAllWindows()


if __name__ == '__main__':
	main()
