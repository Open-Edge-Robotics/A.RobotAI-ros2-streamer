import time
import fire
import zmq
import cv2
import numpy as np

from typing import Any, Generator
from numpy.typing import NDArray


class StreamClient:
    def __init__(self, host: str = "localhost", port: int = 5556):
        context = zmq.Context()

        self.__socket = context.socket(zmq.SUB)
        self.__socket.setsockopt(zmq.SUBSCRIBE, b"")

        self.__socket.connect(f"tcp://{host}:{port}")

        self.__img_size = (848, 480)
        # self.__img_size = (640, 480)

    def __stream(self) -> Generator[NDArray, Any, Any]:
        while True:
            start_time = time.time()
            bytes_img = self.__socket.recv()
            end_time = time.time()
            print("Latency:", end_time - start_time)

            reshape_num = self.__img_size[::-1] + (3,)

            yield np.frombuffer(
                bytes_img,
                dtype=np.uint8,
            ).reshape(reshape_num)

    def run(self):
        try:
            print("Start stream")
            for frame in self.__stream():
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                cv2.imshow("frame", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            cv2.destroyAllWindows()
        except KeyboardInterrupt:
            cv2.destroyAllWindows()
            print("Viewer closed")


if __name__ == "__main__":
    fire.Fire(StreamClient)
