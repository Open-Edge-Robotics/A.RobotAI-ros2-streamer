from typing import Optional
from numpy.typing import NDArray

from ros_stream_viewer.services.viewer_service import BaseStreamViewerService
from ros_stream_viewer.utils import AppLogger


class StreamService(BaseStreamViewerService):
    def __init__(
        self,
        host: str = "localhost",
        port: int = 5556,
        height: int = 480,
        width: int = 640,
        is_frame: bool = True,
        show_network_info: bool = True,
    ) -> None:
        super().__init__(
            logger=AppLogger(),
            host=host,
            port=port,
            height=height,
            width=width,
            is_frame=is_frame,
            show_network_info=show_network_info,
        )

    def _process_frame(self, frame: NDArray) -> Optional[NDArray]:
        """단순 스트림 표시를 위한 프레임 처리"""
        # StreamService는 추가 처리 없이 프레임을 그대로 반환
        return frame

    def _get_window_name(self) -> str:
        """윈도우 이름 반환"""
        return "frame"
