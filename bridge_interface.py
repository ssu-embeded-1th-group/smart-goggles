"""
bridge_interface.py - Arduino Bridge 통신 인터페이스 및 모의(Mock) 계층

Arduino UNO Q의 Linux Python 환경(arduino.app_utils)과
개발용 PC / 테스트 환경 간의 투명한 호환성을 제공합니다.
"""

import sys
from typing import Any, Callable, Dict, Optional

# 실제 Arduino Linux 환경의 arduino.app_utils 임포트 시도
_HAS_REAL_BRIDGE = False
try:
    from arduino.app_utils import App as _RealApp
    from arduino.app_utils import Bridge as _RealBridge
    _HAS_REAL_BRIDGE = True
except ImportError:
    _RealApp = None
    _RealBridge = None


class MockBridge:
    """
    하드웨어가 없거나 테스트 환경에서 동작하는 가상 Arduino Bridge 구현체.
    각 RPC 함수별 응답 핸들러를 등록하여 다양한 시나리오를 모의 테스트할 수 있습니다.
    """

    def __init__(self):
        self._handlers: Dict[str, Callable[..., Any]] = {}
        self.call_history = []
        self._heading: Optional[float] = 0.0  # 통합 스케치의 기본 가상 GPS 시작 방향
        self._setup_default_handlers()

    def _setup_default_handlers(self):
        # 기본 모의 핸들러
        self._handlers["imu_ready"] = lambda: True
        self._handlers["read_imu"] = lambda: "0.010,-0.020,0.980,0.50,-0.10,0.05,25.30"
        self._handlers["read_pitch"] = lambda: 0.58
        self._handlers["read_gps"] = lambda: "37.495000,126.955000"
        self._handlers["read_nav"] = self._read_nav
        self._handlers["set_heading"] = self._set_heading

    def _read_nav(self) -> str:
        state = "GUIDE" if self._heading is not None else "NO_HEADING"
        heading = "" if self._heading is None else str(self._heading)
        return f"1000,{state},0,,{heading},53.3,242.0,37.495000,126.955000,0,1,1"

    def _set_heading(self, heading: float) -> bool:
        """통합 스케치의 인자 1개 규약. 다음 read_nav에서 설정 결과를 볼 수 있다."""
        self._heading = heading
        return True

    def set_handler(self, func_name: str, handler: Callable[..., Any]):
        """특정 RPC 함수 이름에 대한 동작을 가로채거나 재정의합니다."""
        self._handlers[func_name] = handler

    def remove_handler(self, func_name: str):
        if func_name in self._handlers:
            del self._handlers[func_name]

    def call(self, func_name: str, *args, **kwargs) -> Any:
        self.call_history.append((func_name, args, kwargs))
        if func_name in self._handlers:
            return self._handlers[func_name](*args, **kwargs)
        # 등록되지 않은 함수 호출 시 빈 문자열 또는 None 반환
        return ""


class MockApp:
    """
    하드웨어가 없을 때 loop 함수를 단발 또는 제어된 횟수로 실행할 수 있는 Mock App.
    """

    @staticmethod
    def run(user_loop: Callable[[], None], max_iterations: Optional[int] = None):
        """
        테스트 환경에서는 max_iterations 횟수만큼 실행하거나,
        지정되지 않은 경우 무한 루프 대신 1회 실행 후 제어를 반환할 수 있습니다.
        """
        iterations = 0
        try:
            while True:
                user_loop()
                iterations += 1
                if max_iterations is not None and iterations >= max_iterations:
                    break
        except KeyboardInterrupt:
            pass


class BridgeClient:
    """
    실제 Arduino Bridge와 Mock Bridge를 유연하게 전환하여 사용할 수 있는 클라이언트.
    """

    def __init__(self, use_mock: bool = False):
        self._use_mock = use_mock or not _HAS_REAL_BRIDGE
        self.mock_bridge = MockBridge()

    @property
    def is_real_hardware(self) -> bool:
        return _HAS_REAL_BRIDGE and not self._use_mock

    def call(self, func_name: str, *args, **kwargs) -> Any:
        if self.is_real_hardware and _RealBridge is not None:
            return _RealBridge.call(func_name, *args, **kwargs)
        return self.mock_bridge.call(func_name, *args, **kwargs)

    def run_app(self, user_loop: Callable[[], None], max_iterations: Optional[int] = None):
        if self.is_real_hardware and _RealApp is not None:
            _RealApp.run(user_loop=user_loop)
        else:
            MockApp.run(user_loop=user_loop, max_iterations=max_iterations)


# 전역 기본 인스턴스 (main.py에서 직접 Bridge.call() 형태로 사용할 수 있도록 제공)
bridge_client = BridgeClient()
Bridge = bridge_client
