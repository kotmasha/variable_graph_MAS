from __future__ import annotations

import shutil
import subprocess

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

from .base_writer import AnimationWriter


@dataclass(frozen=True)
class GifWriter(AnimationWriter):
    fps: int = 10
    ffmpeg: str = "ffmpeg"

    def __post_init__(self) -> None:
        if self.fps < 1:
            raise ValueError("fps must be at least 1")

        if shutil.which(self.ffmpeg) is None:
            raise FileNotFoundError(
                f"Could not find FFmpeg executable: {self.ffmpeg!r}"
            )

    def save_frames(self, frame_paths: Sequence[Path], output_path: Path, temporary_directory: Path, shape: tuple[int, int]) -> None:
        if not frame_paths:
            raise ValueError("Cannot save an animation with no rendered frames")

        width, height = shape
        palette_path = temporary_directory / "palette.png"

        # First pass: generate a palette from the complete animation.
        palette_command = [
            self.ffmpeg,
            "-y",
            "-loglevel", "error",

            "-f", "rawvideo",
            "-pixel_format", "rgba",
            "-video_size", f"{width}x{height}",
            "-framerate", str(self.fps),
            "-i", "-",

            "-vf", "palettegen",
            "-frames:v", "1",
            str(palette_path),
        ]

        self._run_with_frames(palette_command, frame_paths, "generating GIF palette")

        # Second pass: apply that palette to the animation.
        gif_command = [
            self.ffmpeg,
            "-y",
            "-loglevel", "error",

            "-f", "rawvideo",
            "-pixel_format", "rgba",
            "-video_size", f"{width}x{height}",
            "-framerate", str(self.fps),
            "-i", "-",

            "-i", str(palette_path),

            "-filter_complex", "[0:v][1:v]paletteuse",
            "-loop", "0",
            str(output_path),
        ]

        self._run_with_frames(gif_command,frame_paths,"writing GIF")

    @staticmethod
    def _run_with_frames(command: list[str], frame_paths: Sequence[Path], operation: str) -> None:
        process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )

        if process.stdin is None:
            raise RuntimeError("FFmpeg stdin was not created")

        try:
            for frame_path in frame_paths:
                with frame_path.open("rb") as frame:
                    shutil.copyfileobj(frame, process.stdin)

            process.stdin.close()

            stderr = (
                process.stderr.read().decode("utf-8", errors="replace")
                if process.stderr is not None
                else ""
            )

            return_code = process.wait()

            if return_code != 0:
                raise RuntimeError(
                    f"FFmpeg failed while {operation}:\n"
                    f"{stderr.strip()}"
                )

        except BaseException:
            if process.poll() is None:
                process.kill()

            process.wait()
            raise