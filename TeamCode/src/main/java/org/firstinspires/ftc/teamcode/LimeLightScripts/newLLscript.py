import cv2
import numpy as np

# HSV color ranges for Limelight 3A
# Green: H 40-80, S 60-255, V 60-200
# Purple: H 125-155, S 20-200, V 180-255
GREEN_LOWER = np.array([53,  60,  60])
GREEN_UPPER = np.array([80, 255, 200])
PURPLE_LOWER = np.array([108, 66, 56])
PURPLE_UPPER = np.array([158, 255, 202])

MIN_AREA = 5000.0  # ignore tiny blobs

def _append_target(flat_list, norm_x, norm_y, area, color_id, width, height):
    """Push six floats onto the llpython list."""
    flat_list.extend([
        float(norm_x),  # tx
        float(norm_y),  # ty
        float(area),    # ta
        float(color_id),# ts (repurposed as color ID)
        float(width),   # tl
        float(height)   # th
    ])

def runPipeline(image, llrobot):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    green_mask  = cv2.inRange(hsv, GREEN_LOWER,  GREEN_UPPER)
    purple_mask = cv2.inRange(hsv, PURPLE_LOWER, PURPLE_UPPER)

    kernel = np.ones((5, 5), np.uint8)
    green_mask  = cv2.morphologyEx(green_mask,  cv2.MORPH_OPEN, kernel)
    green_mask  = cv2.morphologyEx(green_mask,  cv2.MORPH_CLOSE, kernel)
    purple_mask = cv2.morphologyEx(purple_mask, cv2.MORPH_OPEN, kernel)
    purple_mask = cv2.morphologyEx(purple_mask, cv2.MORPH_CLOSE, kernel)

    green_contours,  _ = cv2.findContours(green_mask,  cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    purple_contours, _ = cv2.findContours(purple_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    height, width = image.shape[:2]
    all_contours = []
    llpython = []  # flat float list

    def process_contours(contours, color_id, box_color, label):
        for contour in contours:
            area = cv2.contourArea(contour)
            if area <= MIN_AREA:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            cx = x + w // 2
            cy = y + h // 2

            norm_x = (cx - width / 2) / (width / 2)
            norm_y = (height / 2 - cy) / (height / 2)

            cv2.rectangle(image, (x, y), (x + w, y + h), box_color, 2)
            cv2.putText(
                image, label, (x, max(y - 10, 15)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2
            )
            cv2.circle(image, (cx, cy), 5, box_color, -1)

            _append_target(llpython, norm_x, norm_y, area, color_id, w, h)
            all_contours.append(contour)

    process_contours(green_contours, 1.0, (0, 255, 0), "GREEN")
    process_contours(purple_contours, 2.0, (255, 0, 255), "PURPLE")

    # Sort targets by area, descending (six floats per target)
    def target_area(idx):
        return llpython[idx + 2]

    for offset in range(0, len(llpython), 6):
        for compare in range(offset + 6, len(llpython), 6):
            if target_area(compare) > target_area(offset):
                llpython[offset:offset + 6], llpython[compare:compare + 6] = \
                    llpython[compare:compare + 6], llpython[offset:offset + 6]

    green_count = sum(1 for i in range(3, len(llpython), 6) if llpython[i] == 1.0)
    purple_count = sum(1 for i in range(3, len(llpython), 6) if llpython[i] == 2.0)
    cv2.putText(
        image,
        f"Green: {green_count} Purple: {purple_count}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2
    )

    return all_contours, image, llpython