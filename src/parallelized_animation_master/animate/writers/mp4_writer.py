from __future__ import annotations

import shutil
import subprocess

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence
import threading
import time

from .base_writer import AnimationWriter
from ..progress import print_status


@dataclass(frozen=True)
class Mp4Writer(AnimationWriter):
    fps: int = 10
    ffmpeg: str = "ffmpeg"
    ffmpeg_threads: int = 1
    crf: int = 18
    preset: str = "medium"

    def __post_init__(self) -> None:
        if self.fps < 1:
            raise ValueError("fps must be at least 1")

        if self.ffmpeg_threads < 1:
            raise ValueError("ffmpeg_threads must be at least 1")

        if not 0 <= self.crf <= 51:
            raise ValueError("crf must be between 0 and 51")

        if shutil.which(self.ffmpeg) is None:
            raise FileNotFoundError(
                f"Could not find FFmpeg executable: "
                f"{self.ffmpeg!r}"
            )

    def save_frames(
        self,
        frame_paths: Sequence[Path],
        output_path: Path,
        temporary_directory: Path,
        shape: tuple[int, int],
    ) -> None:
        if not frame_paths:
            raise ValueError("Cannot save an animation with no rendered frames")

        width, height = shape

        command = [
            self.ffmpeg,
            "-y",
            "-loglevel", "error",
            "-progress", "pipe:1",
            "-stats_period", "0.25",
            "-nostats",

            "-f", "rawvideo",
            "-pixel_format", "rgba",
            "-video_size", f"{width}x{height}",
            "-framerate", str(self.fps),
            "-i", "-",

            "-an",
            "-c:v", "libx264",
            "-preset", self.preset,
            "-crf", str(self.crf),
            "-threads", str(self.ffmpeg_threads),
            "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2",
            "-pix_fmt", "yuv420p",

            str(output_path),
        ]

        process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        if process.stdin is None:
            raise RuntimeError("FFmpeg stdin was not created")

        started_at = time.perf_counter()
        
        def read_progress() -> None:
            if process.stdout is None:
                return

            for raw_line in process.stdout:
                line = raw_line.decode("utf-8", errors="replace").strip()

                key, _, value = line.partition("=")

                if key == "frame":
                    completed = int(value)
                    elapsed = time.perf_counter() - started_at

                    print_status("compiling mp4", completed, len(frame_paths), elapsed)
            
        progress_thread = threading.Thread(
            target=read_progress,
            daemon=False,
        )
        progress_thread.start()

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
                    f"FFmpeg failed while writing {output_path}:\n"
                    f"{stderr.strip()}"
                )

        except BaseException:
            if process.poll() is None:
                process.kill()

            process.wait()
            raise

        finally:
            if process.stdin is not None and not process.stdin.closed:
                process.stdin.close()

            if process.stdout is not None:
                process.stdout.close()

            if process.stderr is not None:
                process.stderr.close()

            progress_thread.join()