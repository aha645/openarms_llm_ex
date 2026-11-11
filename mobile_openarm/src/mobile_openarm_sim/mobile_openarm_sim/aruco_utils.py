"""ArUco 검출 + 포즈 추정 (OpenCV 4.6 ~ 5.x 양쪽 API 지원).

Ubuntu 24.04 + ROS 2 Jazzy 의 apt python3-opencv 는 4.6 (구 API: detectMarkers),
pip opencv-contrib-python 최신은 4.7+ (신 API: ArucoDetector). 둘 다 동작하도록 감쌌다.
"""
import cv2
import numpy as np


class ArucoDetector:
    def __init__(self, dict_name="DICT_4X4_50", marker_size=0.04, K=None, dist=None):
        self.marker_size = marker_size
        self.K = np.asarray(K, dtype=np.float64) if K is not None else None
        self.dist = np.zeros(5) if dist is None else np.asarray(dist, dtype=np.float64)
        dict_id = getattr(cv2.aruco, dict_name)
        if hasattr(cv2.aruco, "ArucoDetector"):  # OpenCV >= 4.7
            self.dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
            self.params = cv2.aruco.DetectorParameters()
            self._det = cv2.aruco.ArucoDetector(self.dictionary, self.params)
        else:  # OpenCV 4.6
            self.dictionary = cv2.aruco.Dictionary_get(dict_id)
            self.params = cv2.aruco.DetectorParameters_create()
            self._det = None
        self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        # 작은(20~30px) 마커·비스듬한 시야각에서도 검출되도록 완화
        self.params.minMarkerPerimeterRate = 0.01
        self.params.polygonalApproxAccuracyRate = 0.05
        if self._det is not None:
            self._det.setDetectorParameters(self.params)
        h = marker_size / 2
        # 마커 좌표계: 중심 원점, z 가 마커 면에서 밖으로 (OpenCV 관례, 코너 순서 TL,TR,BR,BL)
        self.obj_pts = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)

    def detect(self, rgb):
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        if self._det is not None:
            corners, ids, _ = self._det.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)
        out = []
        if ids is None:
            return out
        for c, i in zip(corners, ids.ravel()):
            item = {"id": int(i), "corners": c.reshape(4, 2)}
            if self.K is not None:
                ok, rvec, tvec = cv2.solvePnP(self.obj_pts, c.reshape(4, 2).astype(np.float64),
                                              self.K, self.dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if ok:
                    R, _ = cv2.Rodrigues(rvec)
                    item["position"] = tvec.reshape(3)
                    item["rotation"] = R
            if "position" in item:
                out.append(item)
        return out

    @staticmethod
    def draw(rgb, dets):
        img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        for d in dets:
            c = d["corners"].astype(int)
            cv2.polylines(img, [c], True, (0, 255, 0), 2)
            cv2.putText(img, f"id {d['id']}", tuple(c[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        return img
