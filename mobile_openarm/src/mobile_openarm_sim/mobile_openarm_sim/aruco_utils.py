"""ArUco 검출 + 포즈 추정 (OpenCV 4.6 ~ 5.x 양쪽 API 지원).

Ubuntu 24.04 + ROS 2 Jazzy 의 apt python3-opencv 는 4.6 (구 API: detectMarkers),
pip opencv-contrib-python 최신은 4.7+ (신 API: ArucoDetector). 둘 다 동작하도록 감쌌다.
"""
import cv2
import numpy as np


class ArucoDetector:
    def __init__(self, dict_name="DICT_4X4_50", marker_size=0.04, K=None, dist=None, scales=(1.0,)):
        """scales: 검출할 영상 배율들. 가까이서 크게 보이는 마커(손목 카메라, 셀 하나가 25 px 넘음)는 원본에서 자주 놓친다 →
        줄인 영상에서도 찾아 원본 좌표로 되돌린다 (손목 카메라 파지 구간 측정: 원본 11/43 → 1, 1/2, 1/3 배 27/43)."""
        self.marker_size = marker_size
        self.scales = tuple(scales)
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
        # 서브픽셀 보정 창 2 px (기본 5): 45 px 마커의 흰 테두리는 3~4 px 라 창이 크면 모서리가 테두리 바깥(상자 윗면 윤곽)으로
        # 끌려갔다 → 방향(yaw) 오차 5° 초과가 검출의 36 %, 최대 33° (머리 카메라, 무작위 배치 40 회 오프라인).
        # 2 px: 최대 1.8°, 깊이 보정 위치 오차 중앙 1.7 → 0.9 mm. 손목 카메라 검출률은 같음 (저장 프레임 354 장)
        self.params.cornerRefinementWinSize = 2
        # 작은(20~30px) 마커·비스듬한 시야각에서도 검출되도록 완화
        self.params.minMarkerPerimeterRate = 0.01
        self.params.polygonalApproxAccuracyRate = 0.05
        if self._det is not None:
            self._det.setDetectorParameters(self.params)
        h = marker_size / 2
        # 마커 좌표계: 중심 원점, z 가 마커 면에서 밖으로 (OpenCV 관례, 코너 순서 TL,TR,BR,BL)
        self.obj_pts = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)

    def _detect_raw(self, gray):
        if self._det is not None:
            corners, ids, _ = self._det.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)
        return [] if ids is None else list(zip(corners, ids.ravel()))

    def detect(self, rgb):
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        found = {}
        for s in self.scales:
            im = gray if s == 1.0 else cv2.resize(gray, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
            for c, i in self._detect_raw(im):
                if int(i) not in found:
                    found[int(i)] = (np.asarray(c, np.float64).reshape(4, 2) + 0.5) / s - 0.5
        out = []
        for i, c in found.items():
            item = {"id": int(i), "corners": c.astype(np.float32)}
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
