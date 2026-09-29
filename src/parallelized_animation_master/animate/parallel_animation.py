from __future__ import annotations

from tempfile import TemporaryDirectory
import time

from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context
from pathlib import Path
from typing import Callable, Generic, Iterable

##################################################
from .scene import AnimationScene
from .progress import _LocalProgress
from .worker_helpers import render_animation_chunk, wait_for_workers, get_frame_size
from .types import FrameT, RenderConfig, RenderChunk, AnimationJob
from .render import AnimationRender

class ParallelAnimation(Generic[FrameT]):
    """
    Render a finite frame sequence using worker-local animation scenes.

    Frames are divided into contiguous chunks. Each worker creates one scene,
    reuses it for its chunk, and writes an intermediate file that is later
    combined by the selected writer.

    Parameters
    ----------
    frames : Iterable[FrameT]
        Frame values passed to ``func``.
    init_func : Callable[[], AnimationScene]
        Creates a worker-local animation scene.
    func : Callable[[FrameT, AnimationScene], None]
        Updates the scene for one frame.
    finalize_func : Callable[[AnimationScene], None] | None, optional
        Finalizes a worker scene after its chunk is rendered.
    config : ParallelConfig | None, optional
        Parallel rendering configuration. Uses defaults when omitted.
    """

    def __init__(self,
        frames: Iterable[FrameT],
        init_func: Callable[[], AnimationScene],
        func: Callable[[FrameT, AnimationScene], None],
        *,
        finalize_func: Callable[[AnimationScene], None] | None = None
    ) -> None:
        """
        Initialize an animation render job.

        Raises
        ------
        TypeError
            If ``init_func``, ``func``, or a provided ``finalize_func`` is not
            callable.
        """
        if not callable(init_func):
            raise TypeError("init_func must be callable")

        if not callable(func):
            raise TypeError("func must be callable")

        if finalize_func is not None and not callable(finalize_func):
            raise TypeError("finalize_func must be callable or None")

        self._frame_source = frames
        self._init_func = init_func
        self._func = func
        self._finalize_func = finalize_func
        self._frames: tuple[FrameT, ...] | None = None

    @property
    def frames(self) -> tuple[FrameT, ...]:
        """
        Materialize and cache the input frame iterable.

        Returns
        -------
        tuple[FrameT, ...]
            Cached frame sequence.

        Raises
        ------
        ValueError
            If the frame iterable is empty.
        """
        if self._frames is None:
            self._frames = tuple(self._frame_source)

            if not self._frames:
                raise ValueError(
                    "ParallelAnimation requires at least one frame"
                )

        return self._frames
    
    def render(self, dpi: int, worker_count: int = 0, temp_root: Path | None = None) -> AnimationRender:
  
        frames = self.frames
        frame_count = len(frames)
        config = RenderConfig(dpi, worker_count, temp_root)

        chunk_count = config.workers

        temporary_directory = TemporaryDirectory(None, "par_anim_", config.temp_root)
        
        tmp_dir = Path(temporary_directory.name)

        job = AnimationJob(self._init_func, self._func, self._finalize_func, config)
    
        chunks = self._partition_frames(frames, chunk_count, tmp_dir)

        started_at = time.perf_counter()
        try:
            self._render_chunks(job, chunks, config.workers)

        except BaseException:
            if temporary_directory is not None:
                temporary_directory.cleanup()
            raise
        
        
        return AnimationRender(
            temporary_directory,
            tuple(
                tmp_dir / f"frame_{index:08d}.rgba"
                for index in range(frame_count)
            ),
            get_frame_size(self._init_func(), config.dpi),
            frame_count,
            config.workers,
            time.perf_counter() - started_at
        )

    def _partition_frames(self, frames: tuple[FrameT, ...], chunk_count: int, temporary_directory: Path) -> list[RenderChunk[FrameT]]:
        quotient, remainder = divmod(len(frames), chunk_count)

        chunks: list[RenderChunk[FrameT]] = []
        begin = 0

        for index in range(chunk_count):
            size = quotient + (index < remainder)
            end = begin + size

            chunks.append(
                RenderChunk(index, begin, frames[begin:end], temporary_directory)
            )

            begin = end

        return chunks

    def _render_chunks(self, job: AnimationJob[FrameT], chunks: list[RenderChunk[FrameT]], worker_count: int) -> list[int]:
        total_frames = sum(len(chunk.frames) for chunk in chunks)

        if worker_count == 1:
            progress = _LocalProgress(total_frames)

            return [
                render_animation_chunk(job, chunk, progress)
                for chunk in chunks
            ]

        context = get_context("spawn")

        with context.Manager() as manager:
            progress_queue = manager.Queue()

            with ProcessPoolExecutor(max_workers=worker_count, mp_context=context) as executor:
                futures = [
                    executor.submit(
                        render_animation_chunk,
                        job,
                        chunk,
                        progress_queue,
                    )
                    for chunk in chunks
                ]

                try:
                    return wait_for_workers(futures, progress_queue, total_frames)

                except BaseException:
                    for future in futures:
                        future.cancel()

                    raise
            

    @staticmethod
    def _normalize_extension(extension: str) -> str:
        """
        Normalize a file extension to lowercase ``.suffix`` form.

        Parameters
        ----------
        extension : str
            Extension with or without a leading period.

        Returns
        -------
        str
            Normalized extension.

        Raises
        ------
        ValueError
            If ``extension`` is empty or whitespace.
        """
        extension = extension.strip().lower()

        if not extension:
            raise ValueError("Output filename must have an extension")

        if not extension.startswith("."):
            extension = f".{extension}"

        return extension
