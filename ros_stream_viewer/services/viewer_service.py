import time
import cv2
import numpy as np
import zmq
from collections import deque
from threading import Event
from abc import ABC, abstractmethod
from typing import Any, Generator, Optional
from numpy.typing import NDArray

from ros_stream_viewer.utils import AppLogger


class BaseStreamViewerService(ABC):
    """스트림 뷰어 서비스의 공통 기능을 제공하는 베이스 클래스"""

    def __init__(
        self,
        logger: AppLogger,
        host: str = "localhost",
        port: int = 5556,
        height: int = 480,
        width: int = 640,
        is_frame: bool = True,
        show_network_info: bool = True,
    ) -> None:
        self._logger = logger.get_logger(name=self.__class__.__name__)
        self._img_size = (width, height)
        self._is_frame = is_frame
        self._show_network_info = show_network_info

        # 소켓 연결
        self._setup_socket(host, port)

        # 네트워크 속도 추적을 위한 변수들 (show_network_info가 False여도 내부적으로는 추적)
        self._bytes_received = 0
        self._frame_count = 0
        self._start_time = time.time()
        self._last_update_time = time.time()
        self._fps_history = deque(maxlen=30)  # 최근 30프레임의 FPS 기록
        self._bytes_history = deque(maxlen=30)  # 최근 30프레임의 바이트 기록

        # 종료 신호를 위한 이벤트
        self._shutdown_event = None
        self._running = True

        # 성능 최적화를 위한 변수들
        self._frame_buffer = None  # 프레임 버퍼 재사용
        self._stats_cache = None  # 통계 캐시
        self._stats_update_interval = 10  # 10프레임마다 통계 업데이트
        self._last_stats_frame = 0

    def _setup_socket(self, host: str, port: int):
        """소켓 연결 설정"""
        try:
            context = zmq.Context()
            self.socket = context.socket(zmq.SUB)
            self.socket.setsockopt(zmq.SUBSCRIBE, b"")
            self.socket.connect(f"tcp://{host}:{port}")
            self._logger.info(f"Connected to {host}:{port}")
        except Exception as e:
            self._logger.error(e)
            raise e

    def set_shutdown_event(self, shutdown_event: Event):
        """종료 이벤트 설정"""
        self._shutdown_event = shutdown_event

    def shutdown(self):
        """서비스 종료"""
        self._running = False
        self._logger.info(f"{self.__class__.__name__} 종료 신호 받음")

    def _stream_frames(self) -> Generator[NDArray, Any, Any]:
        """프레임 스트림 생성기 (공통 로직)"""
        reshape_num = self._img_size[::-1] + (3,)  # 미리 계산

        while self._running and (
            self._shutdown_event is None or not self._shutdown_event.is_set()
        ):
            try:
                start_time = time.time()

                # 논블로킹 소켓 체크 (타임아웃 설정)
                if hasattr(self.socket, "poll"):
                    # ZMQ 소켓인 경우
                    if self.socket.poll(timeout=100):  # 100ms 타임아웃
                        bytes_img = self.socket.recv(flags=0)  # 논블로킹
                    else:
                        continue
                else:
                    bytes_img = self.socket.recv()

                end_time = time.time()

                # 네트워크 속도 추적 (최적화된 버전)
                frame_size = len(bytes_img)
                self._bytes_received += frame_size
                self._frame_count += 1

                # FPS 계산 (매 프레임마다 하지 않고 선택적으로)
                if self._frame_count % 5 == 0:  # 5프레임마다만 계산
                    frame_time = end_time - start_time
                    if frame_time > 0:
                        current_fps = 1.0 / frame_time
                        self._fps_history.append(current_fps)

                # 바이트 기록 추가 (매 프레임마다 하지 않음)
                if self._frame_count % 3 == 0:  # 3프레임마다만 기록
                    self._bytes_history.append(frame_size)

                # 로깅 빈도 줄이기
                if (
                    self._is_frame and self._frame_count % 30 == 0
                ):  # 30프레임마다만 로깅
                    self._logger.info(
                        f"Stream recv time: {end_time - start_time:.4f} seconds, Frame: {self._frame_count}"
                    )

                # 프레임 버퍼 재사용 최적화
                if self._frame_buffer is None or self._frame_buffer.size != len(
                    bytes_img
                ):
                    # NumPy 배열을 쓰기 가능한 상태로 생성 (메모리 재사용)
                    frame_array = np.frombuffer(bytes_img, dtype=np.uint8).reshape(
                        reshape_num
                    )
                    self._frame_buffer = frame_array.copy()
                else:
                    # 기존 버퍼에 데이터 복사 (새로운 할당 없음)
                    np.copyto(
                        self._frame_buffer,
                        np.frombuffer(bytes_img, dtype=np.uint8).reshape(reshape_num),
                    )

                yield self._frame_buffer

            except Exception as e:
                if self._running:
                    self._logger.error(f"스트림 수신 오류: {e}")
                break

    def _get_network_stats(self) -> dict:
        """네트워크 통계 정보를 계산하여 반환 (캐시 사용)"""
        # 통계 업데이트 간격 체크
        if (
            self._stats_cache is not None
            and self._frame_count - self._last_stats_frame < self._stats_update_interval
        ):
            return self._stats_cache

        current_time = time.time()
        elapsed_time = current_time - self._start_time

        # 평균 FPS 계산 (더 효율적인 방법)
        if self._fps_history:
            avg_fps = sum(self._fps_history) / len(self._fps_history)
        else:
            avg_fps = 0

        # 평균 데이터 전송률 계산 (KB/s)
        if elapsed_time > 0:
            avg_throughput_kbps = (self._bytes_received / elapsed_time) / 1024
        else:
            avg_throughput_kbps = 0

        # 최근 프레임 크기 평균 (KB)
        if self._bytes_history:
            avg_frame_size_kb = (
                sum(self._bytes_history) / len(self._bytes_history) / 1024
            )
        else:
            avg_frame_size_kb = 0

        # 기본 통계 (서브클래스에서 확장 가능)
        base_stats = {
            "fps": avg_fps,
            "throughput_kbps": avg_throughput_kbps,
            "frame_size_kb": avg_frame_size_kb,
            "total_frames": self._frame_count,
            "total_mb": self._bytes_received / (1024 * 1024),
        }

        # 서브클래스에서 추가 통계 제공 가능
        additional_stats = self._get_additional_stats()
        base_stats.update(additional_stats)

        # 캐시 업데이트
        self._stats_cache = base_stats
        self._last_stats_frame = self._frame_count

        return self._stats_cache

    def _get_additional_stats(self) -> dict:
        """서브클래스에서 오버라이드할 수 있는 추가 통계"""
        return {}

    def _draw_network_info(self, frame: NDArray) -> NDArray:
        """프레임에 네트워크 정보를 오버레이 (최적화된 버전)"""
        # 프레임이 쓰기 가능한지 확인하고, 필요시 복사
        if not frame.flags.writeable:
            frame = frame.copy()

        # 통계 업데이트는 덜 빈번하게
        stats = self._get_network_stats()

        # 텍스트 정보 준비 (서브클래스에서 커스터마이즈 가능)
        info_lines = self._get_info_lines(stats)

        # 배경 박스 그리기 (한 번에 처리)
        box_height = len(info_lines) * 25 + 20
        box_width = max(300, max(len(line) * 10 for line in info_lines))

        # 검은색 배경
        cv2.rectangle(frame, (10, 10), (box_width, box_height), (0, 0, 0), -1)
        # 녹색 테두리
        cv2.rectangle(frame, (10, 10), (box_width, box_height), (0, 255, 0), 2)

        # 텍스트 그리기 (루프 최적화)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        color = (0, 255, 0)
        thickness = 1

        for i, line in enumerate(info_lines):
            y_pos = 35 + i * 25
            cv2.putText(frame, line, (20, y_pos), font, font_scale, color, thickness)

        return frame

    def _get_info_lines(self, stats: dict) -> list:
        """통계 정보를 텍스트 라인으로 변환 (서브클래스에서 오버라이드 가능)"""
        return [
            f"FPS: {stats['fps']:.1f}",
            f"Throughput: {stats['throughput_kbps']:.1f} KB/s",
            f"Frame Size: {stats['frame_size_kb']:.1f} KB",
            f"Total Frames: {stats['total_frames']}",
            f"Total Data: {stats['total_mb']:.2f} MB",
        ]

    def _handle_frame_processing(self):
        """메인 프레임 처리 루프"""
        try:
            self._logger.info(f"{self.__class__.__name__} 시작됨")

            # 사용법 안내 출력
            info_status = "활성화" if self._show_network_info else "비활성화"
            print(f"\n=== {self.__class__.__name__} 시작 ===")
            print(f"네트워크 정보 표시: {info_status}")
            print("키보드 단축키:")
            print("  'i' 키: 네트워크 정보 표시 토글")
            print("  'q' 키 또는 ESC: 종료")
            print("=" * 40)

            for frame in self._stream_frames():
                if not self._running or (
                    self._shutdown_event and self._shutdown_event.is_set()
                ):
                    break

                # 서브클래스별 프레임 처리
                processed_frame = self._process_frame(frame)
                if processed_frame is not None:
                    # 네트워크 정보 표시 옵션에 따라 오버레이 여부 결정
                    if self._show_network_info:
                        final_frame = self._draw_network_info(processed_frame)
                    else:
                        final_frame = processed_frame

                    # 화면 표시
                    window_name = self._get_window_name()
                    cv2.imshow(window_name, final_frame)

                # 1ms 대기하면서 키 입력 체크
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q") or key == 27:  # 'q' 또는 ESC 키
                    self._logger.info("사용자가 종료를 요청했습니다")
                    break
                elif key == ord("i") or key == ord("I"):  # 'i' 키로 네트워크 정보 토글
                    self._show_network_info = not self._show_network_info
                    status = "활성화" if self._show_network_info else "비활성화"
                    self._logger.info(f"네트워크 정보 표시 {status}")
                    print(f"네트워크 정보 표시 {status} (토글: 'i' 키)")

        except KeyboardInterrupt:
            self._logger.info(f"KeyboardInterrupt - {self.__class__.__name__} 종료")

        except Exception as e:
            self._logger.error(f"{self.__class__.__name__} 실행 중 오류: {e}")
            raise e

        finally:
            cv2.destroyAllWindows()
            self._logger.info(f"{self.__class__.__name__} 정리 완료")
            self.close()

    @abstractmethod
    def _process_frame(self, frame: NDArray) -> Optional[NDArray]:
        """서브클래스에서 구현해야 하는 프레임 처리 메서드"""
        pass

    @abstractmethod
    def _get_window_name(self) -> str:
        """서브클래스에서 구현해야 하는 윈도우 이름 반환 메서드"""
        pass

    def run(self):
        """서비스 실행"""
        self._handle_frame_processing()

    def close(self):
        """리소스 정리"""
        if hasattr(self, "socket"):
            self.socket.close()


class ViewerService(BaseStreamViewerService):
    """기존 호환성을 위한 ViewerService (Deprecated)"""

    def __init__(
        self,
        logger: AppLogger,
        host: str = "localhost",
        port: int = 5556,
    ) -> None:
        # 기존 코드와의 호환성을 위한 간단한 초기화
        self._logger = logger.get_logger(name=__name__)

        try:
            context = zmq.Context()
            self.socket = context.socket(zmq.SUB)
            self.socket.setsockopt(zmq.SUBSCRIBE, b"")
            self.socket.connect(f"tcp://{host}:{port}")

            self._logger.info(f"Connected to {host}:{port}")
        except Exception as e:
            self._logger.error(e)
            raise e

    def _process_frame(self, frame: NDArray) -> Optional[NDArray]:
        """기본 구현 (사용되지 않음)"""
        return frame

    def _get_window_name(self) -> str:
        """기본 윈도우 이름"""
        return "viewer"

    def close(self):
        if hasattr(self, "socket"):
            self.socket.close()
