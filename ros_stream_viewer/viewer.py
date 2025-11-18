import sys
import fire

from ros_stream_viewer.services import StreamService, PredictService
from ros_stream_viewer.utils.thread_helpper import SafeThreadManager


def stream(
    host: str = "localhost",
    port: int = 5556,
    height: int = 480,
    width: int = 640,
    is_frame: bool = True,
    show_network_info: bool = True,
):
    """스트림 뷰어를 안전한 스레드에서 실행

    Args:
        host: 서버 호스트 주소 (기본: localhost)
        port: 서버 포트 번호 (기본: 5556)
        height: 프레임 높이 (기본: 480)
        width: 프레임 너비 (기본: 640)
        is_frame: 프레임 로깅 활성화 여부 (기본: True)
        show_network_info: 네트워크 정보 표시 여부 (기본: True, 실행 중 'i' 키로 토글 가능)
    """
    try:
        stream_svc = StreamService(
            host=host,
            port=port,
            height=height,
            width=width,
            is_frame=is_frame,
            show_network_info=show_network_info,
        )

        manager = SafeThreadManager()
        manager.start(stream_svc)

    except Exception as e:
        print(f"스트림 서비스 시작 실패: {e}")
        sys.exit(1)


def predict(
    host: str = "localhost",
    port: int = 5556,
    height: int = 480,
    width: int = 640,
    is_frame: bool = True,
    show_network_info: bool = True,
):
    """예측 서비스를 안전한 스레드에서 실행

    Args:
        host: 서버 호스트 주소 (기본: localhost)
        port: 서버 포트 번호 (기본: 5556)
        height: 프레임 높이 (기본: 480)
        width: 프레임 너비 (기본: 640)
        is_frame: 프레임 로깅 활성화 여부 (기본: True)
        show_network_info: 네트워크 정보 표시 여부 (기본: True, 실행 중 'i' 키로 토글 가능)
    """
    try:
        predict_svc = PredictService(
            host=host,
            port=port,
            height=height,
            width=width,
            is_frame=is_frame,
            show_network_info=show_network_info,
        )

        manager = SafeThreadManager()
        manager.start(predict_svc)

    except Exception as e:
        print(f"예측 서비스 시작 실패: {e}")
        sys.exit(1)


if __name__ == "__main__":
    fire.Fire(
        {
            "stream": stream,
            "predict": predict,
        }
    )
