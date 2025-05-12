import cv2
import numpy as np
import math

lookahead_distance_pixels = 0.1
frame_skip = 12 # Process every 12th frame
frame_count = 0

### --- Simplified Lane Detection Function --- ###
def find_lane_lines(img, prev_mid_points):
    height, width = img.shape[:2]
    offset = width // 4

    left_midpoints = {}
    right_midpoints = {}

    lane_end_left = False
    lane_end_right = False
     #starting from the bottom row
    for row in reversed(range(height)):
        row_pixels = img[row]
        right_sides, left_sides = [], []

        if row == height - 1:
            start = int(prev_mid_points.get(row, width // 2))
        else:
            below = row + 1
            if below in left_midpoints and below in right_midpoints:
                start = int((left_midpoints[below] + right_midpoints[below]) / 2)
            elif below in right_midpoints:
                start = int(right_midpoints[below] - offset)
            elif below in left_midpoints:
                start = int(left_midpoints[below] + offset)
            else:
                continue

        if not (0 <= start < width):
            break

        # RIGHT LANE
        on_lane = img[row][start].any()
        if on_lane:
            right_sides.append(start)
        for col in range(start, width):
            if not on_lane and img[row][col].any():

                on_lane = True
                right_sides.append(col)
            elif on_lane and not img[row][col].any():

                right_sides.append(col)
                break
        while len(right_sides) < 2:
            right_sides.append(None)

        # LEFT LANE
        on_lane = img[row][start].any()  # or .all() if you want all channels to be > 0
        # if on_lane:
        if on_lane:
            left_sides.append(start)
        for col in reversed(range(0, start)):
            if not on_lane and img[row][col].any():

                on_lane = True
                left_sides.append(col)
            elif on_lane and not img[row][col].any():

                left_sides.append(col)
                break
        while len(left_sides) < 2:
            left_sides.append(None)

        if not lane_end_right and None not in right_sides:
            mid = (right_sides[0] + right_sides[1]) // 2
            right_midpoints[row] = mid
            img[row][mid] = (255, 0, 0)
        elif right_midpoints:
            lane_end_right = True

        if not lane_end_left and None not in left_sides:
            mid = (left_sides[0] + left_sides[1]) // 2
            left_midpoints[row] = mid
            img[row][mid] = (255, 0, 0)
        elif left_midpoints:
            lane_end_left = True

        for point in right_sides + left_sides:
            if point is not None:
                img[row][point] = (255, 255, 0)

    # FILL GAPS
    def extrapolate(midpoints, edge_value):
        if midpoints:
            top = max(midpoints)
            if top - 5 in midpoints:
                slope = (midpoints[top] - midpoints[top - 5]) / 5
                for row in range(top, height):
                    col = int((row - top) * slope + midpoints[top])
                    if 0 <= col < width:
                        midpoints[row] = col
                        img[row][col] = (255, 0, 0)
        else:
            for row in range(height):
                midpoints[row] = edge_value

    extrapolate(right_midpoints, width - 1)
    extrapolate(left_midpoints, 0)
    ### DRAW ROAD PATH AREA ###
    # If we have points on both sides, draw a filled polygon
    if left_midpoints and right_midpoints:
        # Sort by row (top to bottom)
        left_points = [(int(left_midpoints[r]), r) for r in sorted(left_midpoints)]
        right_points = [(int(right_midpoints[r]), r) for r in sorted(right_midpoints, reverse=True)]

        # Combine points into a polygon (left side + reversed right side)
        road_polygon = np.array(left_points + right_points, np.int32)
        if len(road_polygon) >= 3:
            overlay = img.copy()
            cv2.fillPoly(overlay, [road_polygon], (0, 255, 0))
            img = cv2.addWeighted(overlay, 0.3, img, 0.7, 0)

    return [right_midpoints, left_midpoints]
#using the midpoints to find the centre part
def find_good_path(lane_midpoints,img):
    x = []
    y = []
    for row,col in lane_midpoints.items():
        x.append(row)
        y.append(col)
    z= np.polyfit(x, y, 2)
    best_path = np.poly1d(z)
    #draw the function
    for row in range(max(lane_midpoints, key=int)):
        col = int(best_path(row))
        if col < 0 or col >= img.shape[1]:
            continue
        img[row][col] = (0, 255, 0)

    return best_path


def get_midpoints(img, right_lane_mid_points, left_lane_mid_points, right_best_path, left_best_path):
    midpoints = {}  # Dictionary to store the calculated midpoints

    # Loop through each row of the image from top to bottom
    for row in range(img.shape[0]):

        # If the current row is beyond the last known midpoint for either lane,
        # extrapolate the midpoint by drawing a straight line from previous points
        if row > max(right_lane_mid_points, key=int) or row > max(left_lane_mid_points, key=int):
            last_known_row = row - 1

            # If we don't have midpoints for the row 20 rows before, stop the process
            if last_known_row - 20 not in midpoints:
                break

            # Calculate the slope (rate of change) of the midpoint from previous rows
            slope = (midpoints[last_known_row] - midpoints[last_known_row - 20]) / 20

            # Extrapolate the midpoints for all rows below the last known row
            for row in range(last_known_row, img.shape[0]):
                col = (row - last_known_row) * slope + midpoints[last_known_row]   #y = m(x-x1)+y1
                # If the calculated midpoint is outside the image, stop
                if col >= img.shape[1] or col < 0:
                    break
                midpoints[row] = int(col)
                img[row][int(col)] = (255, 0, 0)  # Mark the midpoint in red

            break  # Stop after extrapolating

        # Get the midpoint of the right lane; if not available, use the best fit path
        if row in right_lane_mid_points:
            right_mid_point = right_lane_mid_points[row]
        else:
            right_mid_point = right_best_path(row)

        # Get the midpoint of the left lane; if not available, use the best fit path
        if row in left_lane_mid_points:
            left_mid_point = left_lane_mid_points[row]
        else:
            left_mid_point = left_best_path(row)

        # Calculate the midpoint of the lane as the average of left and right midpoints
        mid_point = int((right_mid_point + left_mid_point) / 2)

        # If the midpoint is outside the image, skip this row
        if mid_point >= img.shape[1] or mid_point < 0:
            continue

        # Save the calculated midpoint
        midpoints[row] = mid_point
        # Mark the midpoint in blue
        img[row][mid_point] = (0, 0, 255)

    return midpoints




def calculate_turn_angle(mid_points, lookahead_distance_pixels):
    # Create lists for the rows (x) and columns (y) from mid_points dictionary
    rows = []
    cols = []

    for row, col in mid_points.items():
        rows.append(row)
        cols.append(col)

    # Create a 3rd-degree polynomial that fits the midpoints (best fit line)
    polynomial_coefficients = np.polyfit(rows, cols, 3)
    best_fit_function = np.poly1d(polynomial_coefficients)

    # Calculate the derivative of the polynomial (the rate of change of the lane path)
    derivative_function = np.polyder(best_fit_function)

    # Use the derivative to find the turn value based on the lookahead distance
    # The derivative at the lookahead distance tells us how much the lane is turning
    turn_angle = -math.atan(derivative_function(lookahead_distance_pixels)) / math.pi / 2  # Normalized angle

    return [turn_angle, best_fit_function]
def find_center_offset(img,best_fit_function, lookahead_distance):
	center_offset = (best_fit_function(lookahead_distance)/img.shape[1])-0.5

	return center_offset
# === Setup ===
scale_down_factor = 0.5
dilate_kernel = np.ones((5, 5), np.uint8)
prev_mid_points = {}
lookahead_dist = 0.1
image_increment = 1

video_path = "D:\\sem6\\dip\\projects\\DIP Project Videos\\PXL_20250325_044603023.TS.mp4"
cap = cv2.VideoCapture(video_path)

if not cap.isOpened():
    print("❌ Error: Could not open video.")
else:
    while True:
        ret, orig_img = cap.read()

        if not ret:
            print("✅ Video ended.")
            break
        frame_count += 1
        if frame_count % frame_skip != 0:
            continue

        # Preprocess
        # Preprocessing (SIMPLIFIED)
        orig_img = cv2.resize(orig_img, (0, 0), fx=scale_down_factor, fy=scale_down_factor)
        gray = cv2.cvtColor(orig_img, cv2.COLOR_BGR2GRAY)
        img = gray.copy()

        # Simple fixed threshold (better visibility)
        _, img = cv2.threshold(img, 150, 255, cv2.THRESH_BINARY)

        # Optional smoothing and dilate to clean up
        img = cv2.GaussianBlur(img, (5, 5), 0)
        img = cv2.dilate(img, dilate_kernel, iterations=1)

        # Convert to 3-channel for drawing
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        lookahead_distance = img.shape[0] * (1 - lookahead_dist)
        print("lookahead_distance = ", lookahead_distance)


        right_mid, left_mid = find_lane_lines(img, prev_mid_points)
        cv2.imshow("Lane Detection", img)
        cv2.waitKey(1)  # or 0 if you want to wait for a keypress

        right_fit = find_good_path(right_mid, img)
        left_fit = find_good_path(left_mid, img)
        mid_points = get_midpoints(img, right_mid, left_mid, right_fit, left_fit)
        #right_lane_mid_points, left_lane_mid_points = find_lane_lines(img, prev_mid_points)

        ### Line of best fit ###


        ### Find mid points of lane ###

        if len(mid_points) == 0:
            print("🛑 STOP — No lane detected.")
            cv2.putText(orig_img, "STOP", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        else:
            print("✅ KEEP GOING FORWARD — Lane detected.")
            turn_value, center_fit = calculate_turn_angle(mid_points, img.shape[0] * (1 - lookahead_dist))
            center_offset = find_center_offset(img, center_fit, lookahead_distance_pixels)

            #new code

            # Decision logic
            # === Draw the center path as a red curve ===
            for row in range(0, img.shape[0], 5):  # skip every few pixels for performance
                col = int(center_fit(row))
                if 0 <= col < img.shape[1]:
                    cv2.circle(orig_img, (col, row), 1, (0, 0, 255), -1)  # red center points

            # === Visual arrow overlay ===
            arrow_start = (img.shape[1] // 2, int(lookahead_distance))
            arrow_length = 60
            arrow_color = (255, 255, 0)

            # === Obstacle Detection ===
            obstacle_detected = False
            obstacle_zones = {"LEFT": False, "CENTER": False, "RIGHT": False}

            # Only process a lookahead region (bottom part of the frame)
            # Only process the very bottom region of the frame
            roi_top = int(img.shape[0] * 0.2)  # skip top 20%
            roi = gray[roi_top:, :]  # take bottom 80%

            # Apply CLAHE for contrast improvement
            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
            roi_enhanced = clahe.apply(roi)

            # Apply edge detection
            edges = cv2.Canny(roi_enhanced, 30, 100)  # reduced thresholds for dim lighting

            # Hough Line Transform
            lines = cv2.HoughLinesP(edges,rho=1, theta=np.pi / 180,threshold=50,minLineLength=50,maxLineGap=10)

            # Draw the lines on the original image
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    # Remember roi was cropped, so shift y by roi_top to match original image coordinates
                    y1 += roi_top
                    y2 += roi_top
                    #cv2.line(orig_img, (x1, y1), (x2, y2), (0, 255, 255), 2)  # draw yellow lines

            # Find contours
            contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            print(f"Contours found: {len(contours)}")


            object_close_threshold = img.shape[0] - 150  # near-bottom zone
            zones = {"LEFT": False, "CENTER": False, "RIGHT": False}

            # Get center X at lookahead row
            lookahead_row = int(img.shape[0] * (1 - lookahead_dist))  # e.g., 80% height
            center_x_path = mid_points.get(lookahead_row, img.shape[1] // 2)

            # Define bands around the center path
            band_width = 60  # You can adjust this to be tighter or wider

            # Band ranges
            left_band = (center_x_path - 3 * band_width, center_x_path - band_width)
            center_band = (center_x_path - band_width, center_x_path + band_width)
            right_band = (center_x_path + band_width, center_x_path + 3 * band_width)

            # For debug visualization (optional)
            cv2.rectangle(orig_img, (left_band[0], roi_top), (left_band[1], img.shape[0]), (255, 0, 0), 1)
            cv2.rectangle(orig_img, (center_band[0], roi_top), (center_band[1], img.shape[0]), (0, 255, 255), 1)
            cv2.rectangle(orig_img, (right_band[0], roi_top), (right_band[1], img.shape[0]), (0, 0, 255), 1)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                x, y, w, h = cv2.boundingRect(cnt)

                x1, y1 = x, y + roi_top
                x2, y2 = x + w, y + h + roi_top
                center_x = x + w // 2

                is_near = y2 > object_close_threshold
                is_valid = area > 800 and h > 80  # Adjust as needed

                if is_near and is_valid:
                    if left_band[0] < center_x < left_band[1]:
                        zones["LEFT"] = True
                        label = "Obstacle: LEFT"
                        color = (255, 0, 0)
                    elif center_band[0] < center_x < center_band[1]:
                        zones["CENTER"] = True
                        label = "Obstacle: CENTER"
                        color = (0, 255, 255)
                    elif right_band[0] < center_x < right_band[1]:
                        zones["RIGHT"] = True
                        label = "Obstacle: RIGHT"
                        color = (0, 0, 255)
                    else:
                        continue  # Outside of known driving zones

                    # Draw and label the obstacle
                    cv2.rectangle(orig_img, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(orig_img, label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # if obstacle_detected:
            #     turn_decision = "STOP 🚫"

            # === Show Output ===
            # cv2.putText(orig_img, f"Decision: {turn_decision}", (40, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)
            # Choose arrow direction based on turn_value
            label = "Analyzing..."
            arrow_end = (arrow_start[0], arrow_start[1] - arrow_length)  # default arrow

            # Obstacle response
            object_ahead = zones["CENTER"]
            object_left = zones["LEFT"]
            object_right = zones["RIGHT"]
            print("Detected Zones:", zones)
            print("Turn value:", turn_value)
            # After
            if any(zones.values()):

                if not object_left and turn_value>0.2:
                    label = "TURN LEFT and No Object is detected in front"
                elif object_left and turn_value>0.2: #in left lane but have object
                    if not object_right:
                        label = "TURN Slightly right and then left as Object is detected in front"
                    elif not object_ahead:
                        label = "TURN Slightly toward centre and then left as Object is detected in front"
                    else:
                        label = "---Stop there is chances of crash---"
                    print("Turn value:", turn_value)

                #logic for right lane scenarios
                elif not object_right and turn_value < -0.2:
                    label = "TURN Right and No Object is detected in front"

                    if object_right and turn_value < -0.2:  # in right lane but have object
                        if not object_left:
                            label = "TURN Slightly left and then right as Object is detected in front"
                        elif not object_ahead:
                            label = "TURN Slightly toward centre and then right as Object is detected in front"
                        else:
                            label = "---Stop there is chances of crash---"
                    print("Turn value:", turn_value)

                #logic for the centre lane
                elif not object_ahead and 0.2 <= turn_value <= -0.2:
                    label = "TURN LEFT and Object is detected in front"
                    if object_ahead and 0.2 <= turn_value <= -0.2:  # in centre lane but have object
                            if not object_right:
                                label = "TURN Slightly right and then towards centre as Object is detected in front"
                            elif not object_left:
                                label = "TURN Slightly toward left and then towrds centre as Object is detected in front"
                            else:
                                label = "---Stop there is chances of crash---"
                    print("Turn value:", turn_value)

                elif object_right and object_left and object_right:
                    label = "Go Straight But can be met with object"
                print("Turn value:", turn_value)

            else :
                # Safe to decide direction based on lane
                if turn_value > 0.2:
                    label = "TURN LEFT (Safe)"

                elif turn_value < -0.2:
                    label = "TURN RIGHT (Safe)"

                else:
                    label = "GO STRAIGHT"

        try:
            cv2.arrowedLine(orig_img, arrow_start, arrow_end, arrow_color, 3, tipLength=0.5)
            cv2.putText(orig_img, label, (50, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, arrow_color, 2)
        except:
            print("Arrow or label not defined in this frame.")
        cv2.imshow("Edges Debug", edges)

        cv2.imshow("Original + Decision", orig_img)
       # cv2.imshow("Driving Output", orig_img)  # show real video
       # cv2.imshow("Lane Detection Threshold", img)  # optional debug

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()