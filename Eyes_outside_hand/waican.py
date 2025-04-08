import cv2
import numpy as np
import os
from typing import Tuple


def CharuCo_calibration(image_file: str = None,
                        AruCo_dict: int = cv2.aruco.DICT_5X5_250,
                        chessboard_size: Tuple[int, int] = (12, 9),
                        square_length: float = 0.03,
                        marker_length: float = 0.022) -> Tuple[np.ndarray, np.ndarray]:
    '''
    Calibrate the camera with CHARUCO chessboard.
    Compute the camera extrinsics and obtain the transformation matrix
    of the camera coordinate system relative to the world coordinate system.

    Args:
        image_file: The path of CharuCo chessboard image file.
        AruCo_dict: The aruco dictionary.
                By default we use the dictionary with 250 corner IDs, 5x5 pixel markers.
        chessboard_size: The number of the horizontal and vertical squares.
        square_length: The length of each square (in meters).
        marker_length: The length of each marker (in meters).

    Returns:
        R: Rotation matrix (3,3)
        T: Translation matrix (3,1)
    '''
    assert image_file.endswith(".png") or image_file.endswith(
        ".jpg"), "Unsupported image type, expect to have png or jpg"
    dictionary = cv2.aruco.getPredefinedDictionary(AruCo_dict)
    # The chessboard size must be horizontal first then vertical.
    board = cv2.aruco.CharucoBoard(chessboard_size, square_length, marker_length, dictionary)
    params = cv2.aruco.DetectorParameters()

    all_charuco_corners = []
    all_charuco_ids = []

    image = cv2.imread(image_file)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(image, dictionary, parameters=params)

    if len(marker_ids) > 0:
        charuco_retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(marker_corners, marker_ids,
                                                                                           image, board)
        if charuco_retval:
            all_charuco_corners.append(charuco_corners)
            all_charuco_ids.append(charuco_ids)

    ret, camera_matrix, dist, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(all_charuco_corners,
                                                                              all_charuco_ids,
                                                                              board,
                                                                              image.shape[::-1],
                                                                              None, None)
    R, _ = cv2.Rodrigues(rvecs[0])
    T = tvecs[0]
    return R, T


def eye_to_hand_calibration():
    '''
    Eye-to-Hand calibration, which maps robot base coordinates in camera coordinate system.
    TODO
    '''


if __name__ == "__main__":
    R, T = CharuCo_calibration("waican_Color.png")
    print(R, T)