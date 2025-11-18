from glob import glob
import os
import time
import cv2
import numpy as np
import torch
from collections import deque, OrderedDict

from typing import List, Optional
from numpy.typing import NDArray

from ultralytics.engine.results import Results
from ultralytics.models import YOLO

from ros_stream_viewer.services.viewer_service import BaseStreamViewerService
from ros_stream_viewer.utils import AppLogger
from ros_stream_viewer.services.obj_class import class_names, class_color


class PredictService(BaseStreamViewerService):
    def __init__(
        self,
        model_name: str = "former_best.pt",
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

        # YOLO 모델 로드
        self.model = self._load_model_with_checkpoint(model_name)
        self.total_time = 0

        # 예측 시간 추적을 위한 추가 변수
        self._predict_time_history = deque(maxlen=30)

        # YOLO 모델 최적화
        if hasattr(self.model, "model"):
            # 추론 속도 최적화 설정
            self.model.overrides["verbose"] = False  # 로깅 최소화
            if torch.cuda.is_available():
                self.model.to("cuda")  # GPU 사용 (가능한 경우)

    def _load_model(self, model: str = "former_best.pt"):
        self._logger.info(f"Loading model: {model}")
        return YOLO(model)

    def _load_model_with_checkpoint(self, model_name: str = "former_best.pt"):
        model = self._load_model(model_name)

        checkpoint_path: str = os.path.join(os.getcwd(), "checkpoints")
        if not os.path.exists(checkpoint_path):
            os.makedirs(checkpoint_path)

        list_of_files = [fname for fname in glob(checkpoint_path + "/*.npz")]

        self._logger.info(f"List of files: {list_of_files}")

        if len(list_of_files) > 0:
            latest_file = max(list_of_files, key=os.path.getctime)
            self._logger.info(f"Latest file: {latest_file}")
            checkpoint_file_path = os.path.join(checkpoint_path, latest_file)

            if os.path.exists(checkpoint_file_path):
                self._logger.info(f"Loading checkpoint from {checkpoint_file_path}")
                weights = np.load(checkpoint_file_path)
                weights = [val for _, val in weights.items()]
                model = self._set_weights(model, weights)

        return model

    def _get_weights(self, model: YOLO) -> List[NDArray]:
        return [val.cpu().numpy() for _, val in model.state_dict().items()]

    def _set_weights(self, model: YOLO, parameters: List[NDArray]):
        params_dict = zip(model.state_dict().keys(), parameters)
        state_dict = OrderedDict({k: torch.tensor(v) for k, v in params_dict})
        model.load_state_dict(state_dict, strict=True)
        return model

    def _get_additional_stats(self) -> dict:
        """PredictService의 추가 통계 (예측 시간)"""
        if self._predict_time_history:
            avg_predict_time = sum(self._predict_time_history) / len(
                self._predict_time_history
            )
        else:
            avg_predict_time = 0

        return {"predict_time_ms": avg_predict_time * 1000}

    def _get_info_lines(self, stats: dict) -> list:
        """PredictService용 정보 라인 (예측 시간 포함)"""
        return [
            f"FPS: {stats['fps']:.1f}",
            f"Throughput: {stats['throughput_kbps']:.1f} KB/s",
            f"Frame Size: {stats['frame_size_kb']:.1f} KB",
            f"Predict Time: {stats.get('predict_time_ms', 0):.1f} ms",
            f"Total Frames: {stats['total_frames']}",
            f"Total Data: {stats['total_mb']:.2f} MB",
        ]

    def _process_frame(self, frame: NDArray) -> Optional[NDArray]:
        """YOLO 예측을 수행하고 결과를 시각화"""
        predict_start_time = time.time()
        # YOLO 추론 최적화: 불필요한 verbose 출력 억제
        results: List[Results] = self.model(frame, verbose=False)
        predict_end_time = time.time()

        # 예측 시간 기록 (매 프레임마다 하지 않음)
        predict_time = predict_end_time - predict_start_time
        if self._frame_count % 3 == 0:  # 3프레임마다만 기록
            self._predict_time_history.append(predict_time)

        # 로깅 빈도 줄이기
        if self._is_frame and self._frame_count % 30 == 0:  # 30프레임마다만 로깅
            self.total_time += predict_time
            self._logger.info(f"Predict time: {predict_time:.4f} seconds")
            self._logger.info(f"Total time: {self.total_time:.4f} seconds")
            self.total_time = 0

        # 결과 처리 최적화 (보통 결과는 1개)
        if results:
            result = results[0]  # 첫 번째 결과만 처리
            return self._get_img(result)

        return None

    def _get_img(self, result: Results) -> Optional[NDArray]:
        """결과 이미지 생성 (최적화된 버전)"""
        img = result.orig_img.copy()
        if result.boxes is None or len(result.boxes) == 0:
            return img  # 빈 이미지라도 반환 (None 대신)

        # 박스 그리기 최적화
        boxes = result.boxes
        for box in boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls_id = int(box.cls[0])

            # 배열 범위 체크 생략 (성능 향상)
            if 0 <= cls_id < len(class_names):
                obj_name = class_names[cls_id]
                conf = float(box.conf[0])
                color = class_color[cls_id]

                # 사각형과 텍스트 한 번에 그리기
                cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
                cv2.putText(
                    img,
                    f"{obj_name}, {conf:.2f}",
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    color,
                    2,
                )
        return img

    def _get_window_name(self) -> str:
        """윈도우 이름 반환"""
        return "viewer"
