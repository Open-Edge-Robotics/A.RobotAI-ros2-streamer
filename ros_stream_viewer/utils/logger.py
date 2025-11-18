import logging

from typing import Optional


class AppLogger:

    def __init__(
        self,
        level: str = "INFO",
    ) -> None:
        self._level = level

    def get_logger(
        self,
        name: Optional[str] = None,
    ) -> logging.Logger:
        logger = logging.getLogger(name if name else __name__)
        logger.setLevel(level=self._level)

        if logger.hasHandlers:
            logger.handlers.clear()

        formatter = logging.Formatter(
            "%(asctime)s - %(name)s - %(levelname)s - %(message)s - %(filename)s:%(lineno)d"
        )

        stream_handler = logging.StreamHandler()
        stream_handler.setFormatter(formatter)

        logger.addHandler(stream_handler)
        return logger
