from glob import glob
import os
import cv2
import numpy as np
import torch

from rclpy.impl.rcutils_logger import RcutilsLogger

from numpy.typing import NDArray
from collections import OrderedDict

from ultralytics.engine.results import Results
from ultralytics.models import YOLO

from typing import List, Optional

from .obj_class import class_names, class_color


class StreamPredictService:
    def __init__(
        self,
        logger: RcutilsLogger,
        model_name: str = "former_best.pt",
    ) -> None:
        self._logger = logger
        self.model = self._load_model_with_checkpoint(model_name)

    def _load_model(self, model: str = "former_best.pt") -> YOLO:
        self._logger.info("Loading model: {model}")
        return YOLO(model)

    def _load_model_with_checkpoint(
        self, model_name: str = "former_best.pt"
    ) -> Optional[YOLO]:
        try:
            model = self._load_model(model_name)

            checkpoint_path: str = os.path.join(os.getcwd(), "checkpoints")
            if not os.path.exists(checkpoint_path):
                os.makedirs(checkpoint_path)

            if os.getenv("KUBERNETES_SERVICE_HOST") is not None:
                checkpoint_path = "/mnt/checkpoints/"

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

        except FileNotFoundError as e:
            self._logger.error(f"former_best.pt not found: {e}")
            return None

        except Exception as e:
            self._logger.error(e)
            return None

    def _get_weights(self, model: YOLO) -> List[NDArray]:
        return [val.cpu().numpy() for _, val in model.state_dict().items()]

    def _set_weights(self, model: YOLO, parameters: List[NDArray]):
        params_dict = zip(model.state_dict().keys(), parameters)
        state_dict = OrderedDict({k: torch.tensor(v) for k, v in params_dict})
        model.load_state_dict(state_dict, strict=True)
        return model

    def _get_img(self, result: Results) -> Optional[NDArray]:
        img = result.orig_img.copy()
        if result.boxes is None:
            return

        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls_id = int(box.cls[0])
            obj_name = class_names[cls_id]
            conf = box.conf[0]
            cv2.rectangle(img, (x1, y1), (x2, y2), class_color[cls_id], 2)
            cv2.putText(
                img,
                f"{obj_name}, {conf:.2f}",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                class_color[cls_id],
                2,
            )

        return img

    def predict(self, frame: NDArray) -> Optional[NDArray]:
        try:
            if self.model is None:
                self._logger.error("Model is None")
                return None

            results: List[Results] = self.model(frame)
            for i, r in enumerate(results):
                img = self._get_img(r)
                if img is None:
                    self._logger.warning(f"Result image {i} is None")
                    continue
                return img
        except Exception as e:
            self._logger.error(e)
            raise e

        return None
