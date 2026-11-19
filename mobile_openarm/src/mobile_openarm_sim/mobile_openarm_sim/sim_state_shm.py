"""시뮬 상태 공유 메모리 (브리지 → 카메라 서버).

카메라 렌더링·ArUco 검출을 물리 스레드에서 하면 그 시간만큼 GIL 을 쥐어 물리가 멈춘다 (검증 기록 11절의 브리지 지연과 같은 종류).
그래서 브리지는 렌더링에 필요한 상태만 공유 메모리에 쓰고, 별도 프로세스(camera_server.py)가 같은 MJCF 로 장면을 다시 만들어 그린다.
ROS 토픽(/joint_states 등)으로 넘기면 상자·사람 자세와 시점이 어긋나고, 직렬화·DDS 비용이 크다.

배치 (float64): [seq, time, tilt_h, tilt_pitch, tilt_roll, qpos(nq), mocap_pos(nmocap*3), mocap_quat(nmocap*4)]
seqlock: 쓰는 쪽은 seq 를 홀수로 만들고 쓴 뒤 짝수로. 읽는 쪽은 seq 가 짝수이고 읽기 전후가 같을 때만 받아들인다.
"""
from multiprocessing import shared_memory

import numpy as np

HEADER = 5
DEFAULT_NAME = "openarm_sim_state"


def size_of(m):
    return HEADER + m.nq + m.nmocap * 7


class StateWriter:
    def __init__(self, m, name=DEFAULT_NAME):
        n = size_of(m)
        try:                                    # 이전 실행이 남긴 블록은 지우고 새로 (크기가 다를 수 있음)
            old = shared_memory.SharedMemory(name=name)
            old.close()
            old.unlink()
        except FileNotFoundError:
            pass
        self.shm = shared_memory.SharedMemory(name=name, create=True, size=n * 8)
        self.a = np.ndarray((n,), np.float64, buffer=self.shm.buf)
        self.a[:] = 0
        self.nq, self.nm = m.nq, m.nmocap

    def write(self, d, tilt):
        a = self.a
        a[0] += 1                               # 홀수: 쓰는 중
        a[1] = d.time
        a[2:5] = tilt
        o = HEADER
        a[o:o + self.nq] = d.qpos
        o += self.nq
        a[o:o + 3 * self.nm] = d.mocap_pos.ravel()
        o += 3 * self.nm
        a[o:o + 4 * self.nm] = d.mocap_quat.ravel()
        a[0] += 1                               # 짝수: 다 씀

    def close(self):
        self.shm.close()
        try:
            self.shm.unlink()
        except FileNotFoundError:
            pass


class StateReader:
    def __init__(self, m, name=DEFAULT_NAME):
        self.shm = shared_memory.SharedMemory(name=name)
        n = size_of(m)
        if self.shm.size < n * 8:
            raise RuntimeError(f"공유 메모리 {name} 크기 {self.shm.size} < 필요 {n * 8} (브리지와 카메라 서버의 MJCF 가 다름)")
        self.a = np.ndarray((n,), np.float64, buffer=self.shm.buf)
        self.nq, self.nm = m.nq, m.nmocap
        self.last_seq = -1

    def read(self, d):
        """새 상태가 있으면 d 에 넣고 (time, tilt) 반환, 없거나 쓰는 중이면 None."""
        for _ in range(5):
            s0 = self.a[0]
            if s0 % 2 or s0 == self.last_seq or s0 == 0:
                if s0 % 2:
                    continue
                return None
            snap = self.a.copy()
            if self.a[0] != s0:
                continue
            self.last_seq = s0
            o = HEADER
            d.qpos[:] = snap[o:o + self.nq]
            o += self.nq
            d.mocap_pos[:] = snap[o:o + 3 * self.nm].reshape(-1, 3)
            o += 3 * self.nm
            d.mocap_quat[:] = snap[o:o + 4 * self.nm].reshape(-1, 4)
            return float(snap[1]), tuple(snap[2:5])
        return None

    def close(self):
        self.shm.close()
