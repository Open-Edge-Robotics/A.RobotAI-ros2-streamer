import signal
import sys
from threading import Thread, Event


class SafeThreadManager:
    """안전한 스레드 관리를 위한 클래스"""

    def __init__(self):
        self.shutdown_event = Event()
        self.thread = None
        self.service = None

    def setup_signal_handlers(self):
        """시그널 핸들러 설정"""

        def signal_handler(signum, frame):
            print(f"\n시그널 {signum} 수신. 안전하게 종료 중...")
            self.shutdown()
            sys.exit(0)

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

    def run_service(self, service):
        """서비스를 안전한 스레드에서 실행"""
        self.service = service

        # 서비스에 종료 이벤트 전달 (만약 서비스가 지원한다면)
        if hasattr(service, "set_shutdown_event"):
            service.set_shutdown_event(self.shutdown_event)

        try:
            service.run()
        except Exception as e:
            print(f"서비스 실행 중 오류 발생: {e}")
            self.shutdown()
            raise

    def start(self, service):
        """스레드를 시작하고 서비스 실행"""
        self.setup_signal_handlers()

        self.thread = Thread(
            target=self.run_service,
            args=(service,),
            daemon=True,  # 데몬 스레드로 설정
            name=f"{service.__class__.__name__}Thread",
        )

        print(f"{service.__class__.__name__} 시작 중...")
        self.thread.start()

        try:
            # 메인 스레드에서 대기하면서 종료 신호 모니터링
            while not self.shutdown_event.is_set() and self.thread.is_alive():
                self.thread.join(timeout=0.5)

        except KeyboardInterrupt:
            print("\nKeyboardInterrupt 감지. 종료 중...")
            self.shutdown()

        finally:
            self.cleanup()

    def shutdown(self):
        """종료 신호 설정"""
        print("종료 신호 설정 중...")
        self.shutdown_event.set()

        # 서비스에 종료 신호 전달
        if self.service and hasattr(self.service, "shutdown"):
            self.service.shutdown()

    def cleanup(self):
        """리소스 정리"""
        if self.thread and self.thread.is_alive():
            print("스레드 정리 중...")
            self.thread.join(timeout=2.0)

        if self.service and hasattr(self.service, "close"):
            print("서비스 리소스 정리 중...")
            self.service.close()

        print("정리 완료.")
