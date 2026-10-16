#!/usr/bin/env python3
"""
OmniMow stereo calibration (runs automatically on first boot, see start_vision.sh).

How to use
  1. Print a checkerboard with 10 x 7 inner corners (11 x 8 squares), 25 mm squares, glue it flat
     on cardboard. Other boards: set OMNIMOW_CALIB_BOARD_COLS/ROWS/SQUARE in omnimow.env.
  2. Hold it 0.5 - 2 m in front of the mower so BOTH cameras see the whole board.
  3. Move it to a new position - left/right, near/far, out into the corners of the image, tilted
     in different directions - and hold it still for a moment. A frame is taken automatically
     once the board stands still in a new position (the cameras free-run, so a moving board
     would sit in different places in the left and right image). Progress in the log and in
     /opt/omnimow/calibration/status.json, which the app shows.
  4. When enough views are collected the calibration is computed, checked and saved as ROS
     camera_info files. The stereo node then starts with rectified images.

Re-calibrate: delete /opt/omnimow/calibration/left.yaml (or use the app) and restart stereo_vision.
Offline: python3 calibrate_stereo.py --images <dir with left_XXX.png / right_XXX.png>
"""
import argparse
import glob
import json
import os
import time

import cv2
import numpy as np

CALIB_DIR = os.getenv('OMNIMOW_CALIB_DIR', '/calibration')


# ---------------------------------------------------------------------------- core functions
def board_points(cols, rows, square):
    """3D corner coordinates of the checkerboard in its own plane (z = 0), in metres."""
    pts = np.zeros((rows * cols, 3), np.float32)
    pts[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    return pts


def find_corners(gray, cols, rows):
    """Checkerboard corners with sub-pixel accuracy, or None."""
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
    found, corners = cv2.findChessboardCorners(gray, (cols, rows), flags)
    if not found:
        return None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.001)
    return cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)


def is_new_view(corners, accepted, min_shift_px):
    """True when the board has moved enough since every accepted view (avoids near-duplicates)."""
    pts = corners.reshape(-1, 2)
    for prev in accepted:
        if np.mean(np.linalg.norm(pts - prev.reshape(-1, 2), axis=1)) < min_shift_px:
            return False
    return True


def calibrate(object_points, left_points, right_points, image_size, model="rational"):
    """Mono calibration of each camera, then stereo calibration and rectification.
    model: "rational" (8 coefficients, rational_polynomial: follows the strong distortion in the
    corners of the IMX219s' full ~72 degree field of view) or "standard" (5, plumb_bob)."""
    rational = model == "rational"
    flags = cv2.CALIB_RATIONAL_MODEL if rational else 0
    n = 8 if rational else 5  # Passed in, or OpenCV returns 5 coefficients and the stereo step breaks
    rms_l, K1, D1, _, _ = cv2.calibrateCamera(object_points, left_points, image_size, None, np.zeros(n),
                                              flags=flags)
    rms_r, K2, D2, _, _ = cv2.calibrateCamera(object_points, right_points, image_size, None, np.zeros(n),
                                              flags=flags)
    # Some OpenCV versions return 14 coefficients; past the first n they stay 0 with these flags,
    # and camera_info's rational_polynomial has 8
    D1, D2 = D1.ravel()[:n], D2.ravel()[:n]
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
    rms, K1, D1, K2, D2, R, T, _, _ = cv2.stereoCalibrate(
        object_points, left_points, right_points, K1, D1, K2, D2, image_size,
        criteria=criteria, flags=cv2.CALIB_FIX_INTRINSIC | flags)
    D1, D2 = D1.ravel()[:n], D2.ravel()[:n]
    # alpha=0: crop to the area valid in both images (no black borders in the disparity)
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(K1, D1, K2, D2, image_size, R, T, alpha=0)
    return {
        "image_size": image_size, "K1": K1, "D1": D1, "K2": K2, "D2": D2, "R": R, "T": T,
        "R1": R1, "R2": R2, "P1": P1, "P2": P2, "Q": Q,
        "rms_left": rms_l, "rms_right": rms_r, "rms_stereo": rms,
        "baseline": float(np.linalg.norm(T)), "focal_px": float(P1[0, 0]),
    }


def camera_info_dict(name, image_size, K, D, R, P):
    """camera_info YAML in the format of ROS camera_calibration_parsers."""
    D = np.asarray(D).ravel()
    return {
        "image_width": int(image_size[0]), "image_height": int(image_size[1]),
        "camera_name": name,
        "camera_matrix": {"rows": 3, "cols": 3, "data": [float(v) for v in np.asarray(K).ravel()]},
        "distortion_model": "rational_polynomial" if len(D) > 5 else "plumb_bob",
        "distortion_coefficients": {"rows": 1, "cols": len(D), "data": [float(v) for v in D]},
        "rectification_matrix": {"rows": 3, "cols": 3, "data": [float(v) for v in np.asarray(R).ravel()]},
        "projection_matrix": {"rows": 3, "cols": 4, "data": [float(v) for v in np.asarray(P).ravel()]},
    }


def validate(result, expected_baseline, max_rms):
    """
    Returns (error, retry). retry=True: collect new views (blurry/moving board).
    retry=False: a setup problem that new views cannot fix.
    """
    if result["T"][0, 0] > 0 and abs(result["T"][0, 0]) >= 0.8 * result["baseline"]:
        return "Left and right camera are swapped (swap OMNIMOW_CAMERA_DEVICE_LEFT/RIGHT)", False
    if result["rms_stereo"] > max_rms:
        return f"Reprojection error {result['rms_stereo']:.2f} px > {max_rms} px (board not flat or blurry images?)", True
    if abs(result["T"][0, 0]) < 0.8 * result["baseline"]:
        return "Cameras are not side by side (check OMNIMOW_CAMERA_DEVICE_LEFT/RIGHT)", False
    if expected_baseline > 0 and abs(result["baseline"] - expected_baseline) > 0.3 * expected_baseline:
        return (f"Baseline {result['baseline'] * 1000:.0f} mm differs > 30% from the expected "
                f"{expected_baseline * 1000:.0f} mm (check OMNIMOW_CALIB_BOARD_SQUARE)"), False
    return None, False


def save(result, out_dir):
    import yaml
    os.makedirs(out_dir, exist_ok=True)
    size = result["image_size"]
    for name, K, D, R, P in [("left", result["K1"], result["D1"], result["R1"], result["P1"]),
                             ("right", result["K2"], result["D2"], result["R2"], result["P2"])]:
        tmp = os.path.join(out_dir, f".{name}.yaml.tmp")
        with open(tmp, "w") as f:
            yaml.safe_dump(camera_info_dict(f"omnimow_{name}", size, K, D, R, P), f, sort_keys=False)
        os.replace(tmp, os.path.join(out_dir, f"{name}.yaml"))
    summary = {k: result[k] for k in ("rms_left", "rms_right", "rms_stereo", "baseline", "focal_px")}
    summary.update({"image_size": list(size), "date": time.strftime("%Y-%m-%d %H:%M:%S")})
    with open(os.path.join(out_dir, "stereo_summary.json"), "w") as f:
        json.dump(summary, f, indent=2)


# ---------------------------------------------------------------------------- runtime
class Status:
    def __init__(self, out_dir, needed):
        self.path = os.path.join(out_dir, "status.json")
        self.needed = needed
        os.makedirs(out_dir, exist_ok=True)

    def write(self, state, samples=0, message=""):
        data = {"state": state, "samples": samples, "needed": self.needed, "message": message,
                "updated": time.strftime("%Y-%m-%d %H:%M:%S")}
        with open(self.path + ".tmp", "w") as f:
            json.dump(data, f)
        os.replace(self.path + ".tmp", self.path)
        print(f"[calibration] {state}: {samples}/{self.needed} {message}", flush=True)


def image_pairs_from_dir(path):
    for left in sorted(glob.glob(os.path.join(path, "left_*.png"))):
        right = left.replace("left_", "right_")
        if os.path.exists(right):
            yield cv2.imread(left), cv2.imread(right)


def image_pairs_from_cameras(width, height):
    from camera import StereoCapture
    cameras = StereoCapture(width=width, height=height)
    try:
        while True:
            # Tight 10 ms pairing: the board may be moving, and a time offset between the two
            # images would look like a wrong camera geometry
            left, right = cameras.read(timeout=2.0, max_skew=0.01)
            if left is not None:
                yield left, right
    finally:
        cameras.release()


def run(pairs, cols, rows, square, needed, expected_baseline, max_rms, out_dir, min_shift_px,
        model="rational", still_px=None):
    """still_px: only take a view once the board has moved less than this between two frames
    (live cameras: they free-run, so left and right are taken a few ms apart, and a moving
    board then sits in different places in the two images). None for stored image pairs."""
    status = Status(out_dir, needed)
    obj = board_points(cols, rows, square)
    samples_dir = os.path.join(out_dir, "samples")
    os.makedirs(samples_dir, exist_ok=True)
    left_pts, right_pts, image_size = [], [], None
    status.write("waiting_for_board", 0, f"Hold a {cols}x{rows} checkerboard in front of both cameras")
    last_status = 0.0
    previous = None  # Left corners of the previous frame (stillness check)

    for left, right in pairs:
        image_size = (left.shape[1], left.shape[0])
        gray_l = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        c_l = find_corners(gray_l, cols, rows)
        moving = False
        if still_px is not None:
            moving = (c_l is None or previous is None
                      or np.mean(np.linalg.norm(c_l.reshape(-1, 2) - previous.reshape(-1, 2), axis=1)) > still_px)
            previous = c_l
        c_r = find_corners(gray_r, cols, rows) if c_l is not None and not moving else None
        if c_l is not None and c_r is not None and is_new_view(c_l, left_pts, min_shift_px):
            left_pts.append(c_l)
            right_pts.append(c_r)
            n = len(left_pts)
            cv2.imwrite(os.path.join(samples_dir, f"left_{n:03d}.png"), left)
            cv2.imwrite(os.path.join(samples_dir, f"right_{n:03d}.png"), right)
            status.write("capturing", n, "Board captured - move it to a new position/angle")
        elif time.time() - last_status > 10:
            last_status = time.time()
            message = ("Board not visible in both cameras" if c_l is None
                       else "Hold the board still" if moving
                       else "Board not visible in both cameras" if c_r is None else "Move the board")
            status.write("capturing" if left_pts else "waiting_for_board", len(left_pts), message)

        if len(left_pts) >= needed:
            status.write("calibrating", len(left_pts))
            result = calibrate([obj] * len(left_pts), left_pts, right_pts, image_size, model)
            error, retry = validate(result, expected_baseline, max_rms)
            if error is None:
                save(result, out_dir)
                status.write("done", len(left_pts),
                             f"RMS {result['rms_stereo']:.2f} px (left {result['rms_left']:.2f}, right "
                             f"{result['rms_right']:.2f}), baseline {result['baseline'] * 1000:.1f} mm, "
                             f"focal {result['focal_px']:.0f} px, {model} model")
                return result
            if not retry:
                status.write("failed", len(left_pts), error)
                return None
            # Start over: probably bad views (blur, moving board) - better to collect fresh ones
            status.write("retrying", 0, error)
            left_pts, right_pts = [], []

    status.write("failed", len(left_pts), "Ran out of images before enough views were collected")
    return None


def main():
    parser = argparse.ArgumentParser(description="OmniMow stereo calibration")
    parser.add_argument("--images", help="Directory with left_XXX.png/right_XXX.png instead of the cameras")
    parser.add_argument("--out", default=CALIB_DIR)
    args = parser.parse_args()

    width = int(os.getenv("OMNIMOW_CAMERA_WIDTH", "640"))
    height = int(os.getenv("OMNIMOW_CAMERA_HEIGHT", "360"))
    pairs = image_pairs_from_dir(args.images) if args.images else image_pairs_from_cameras(width, height)
    result = run(
        pairs,
        cols=int(os.getenv("OMNIMOW_CALIB_BOARD_COLS", "10")),
        rows=int(os.getenv("OMNIMOW_CALIB_BOARD_ROWS", "7")),
        square=float(os.getenv("OMNIMOW_CALIB_BOARD_SQUARE", "0.025")),
        needed=int(os.getenv("OMNIMOW_CALIB_SAMPLES", "30")),
        expected_baseline=float(os.getenv("OMNIMOW_CAMERA_BASELINE", "0.06")),
        max_rms=float(os.getenv("OMNIMOW_CALIB_MAX_RMS", "1.0")),
        out_dir=args.out,
        min_shift_px=width * 0.04,
        model=os.getenv("OMNIMOW_CALIB_MODEL", "rational").lower(),
        still_px=None if args.images else float(os.getenv("OMNIMOW_CALIB_STILL_PX", "0.5")),
    )
    raise SystemExit(0 if result else 1)


if __name__ == "__main__":
    main()
