from __future__ import annotations

from abc import ABC, abstractmethod
from pathlib import Path
from typing import Sequence


class AnimationWriter(ABC):
    """Configured output-format writer for rendered animation frames."""

    @abstractmethod
    def save_frames(self, frame_paths: Sequence[Path], output_path: Path, temporary_directory: Path, shape: tuple[int, int]) -> None:
        """
        Save rendered frames to the requested output format.

        Parameters
        ----------
        frame_paths : Sequence[Path]
            Ordered paths to the rendered frame images.
        output_path : Path
            Destination output path.
        temporary_directory : Path
            Temporary directory available for writer-specific intermediate files.
        shape: TODO
        """