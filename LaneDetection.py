"""Lane detection with perspective warp and sliding-window curve fitting.

Examples:
    python LaneDetection.py --video project_video.mp4
    python LaneDetection.py --camera 1 --arduino
"""

import argparse

import cv2
import numpy as np

from utlis import (drawLines, drawPoints, draw_lanes, get_curve, initializeTrackbars, perspective_warp,
                   sliding_window, stackImages, thresholding, undistort, valTrackbars)

FRAME_W, FRAME_H = 640, 480
SMOOTHING_FRAMES = 10
LEFT_THRESHOLD, RIGHT_THRESHOLD = -50, 60
# Initial trackbar values for the warp trapezoid: widthTop, heightTop, widthBottom, heightBottom
TRACKBARS_CAMERA = [24, 55, 12, 100]
TRACKBARS_VIDEO = [42, 63, 14, 87]


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = p.add_mutually_exclusive_group()
    src.add_argument("--video", default="project_video.mp4", help="video file to process")
    src.add_argument("--camera", type=int, help="camera index to use instead of a video")
    p.add_argument("--arduino", action="store_true", help="send L/R/F steering commands over serial")
    p.add_argument("--no-pipeline", action="store_true", help="hide the debug pipeline window")
    return p.parse_args()


def steering(curve: int) -> str:
    if curve < LEFT_THRESHOLD:
        return "L"
    if curve > RIGHT_THRESHOLD:
        return "R"
    return "F"


def main():
    args = parse_args()
    use_camera = args.camera is not None
    send = None
    if args.arduino:
        from serial_test import Send, close
        send = Send

    cap = cv2.VideoCapture(args.camera if use_camera else args.video)
    if not cap.isOpened():
        raise SystemExit(f"Could not open {'camera ' + str(args.camera) if use_camera else args.video}")
    if use_camera:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

    initializeTrackbars(TRACKBARS_CAMERA if use_camera else TRACKBARS_VIDEO)
    history = np.zeros(SMOOTHING_FRAMES)
    idx = 0

    try:
        while True:
            ok, img = cap.read()
            if not ok or img is None:
                break  # end of video or camera disconnected
            img = cv2.resize(img, (FRAME_W, FRAME_H))
            img_points = img.copy()
            img_final = img.copy()

            img_undist = undistort(img)
            img_thres, img_canny, img_color = thresholding(img_undist)
            src = valTrackbars()
            img_warp = perspective_warp(img_thres, dst_size=(FRAME_W, FRAME_H), src=src)
            img_points = drawPoints(img_points, src)
            img_sliding, curves, lanes, ploty = sliding_window(img_warp, draw_windows=True)

            lane_curve = 0
            try:
                curverad = get_curve(img_final, curves[0], curves[1])
                lane_curve = np.mean([curverad[0], curverad[1]])
                img_final = draw_lanes(img, curves[0], curves[1], FRAME_W, FRAME_H, src=src)

                current = lane_curve // 50
                average = current if int(np.sum(history)) == 0 else np.sum(history) // history.shape[0]
                history[idx] = average if abs(average - current) > 200 else current
                idx = (idx + 1) % SMOOTHING_FRAMES

                direction = steering(int(average))
                cv2.putText(img_final, f"{int(average)} {direction}", (FRAME_W // 2 - 70, 70),
                            cv2.FONT_HERSHEY_DUPLEX, 1.75, (0, 0, 255), 2, cv2.LINE_AA)
                if send:
                    send(direction)
            except (IndexError, TypeError, ValueError, np.linalg.LinAlgError):
                pass  # no lane found in this frame

            img_final = drawLines(img_final, lane_curve)
            if not args.no_pipeline:
                img_thres_bgr = cv2.cvtColor(img_thres, cv2.COLOR_GRAY2BGR)
                stacked = stackImages(0.7, ([img, img_undist, img_points],
                                            [img_color, img_canny, img_thres_bgr],
                                            [img_warp, img_sliding, img_final]))
                cv2.imshow("Pipeline", stacked)
            cv2.imshow("Result", img_final)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        if send:
            close()


if __name__ == "__main__":
    main()
