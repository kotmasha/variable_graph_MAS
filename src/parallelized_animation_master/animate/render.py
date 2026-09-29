from __future__ import annotations

import os
from dataclasses import dataclass

from pathlib import Path
from tempfile import TemporaryDirectory

import os
from tempfile import TemporaryDirectory, mkstemp

from pathlib import Path
from typing import Any
##################################################
from .types import  AnimationResult
from .writers import AnimationWriter, make_writer, WRITERS

@dataclass
class AnimationRender:
    _temporary_directory: TemporaryDirectory
    frame_paths: tuple[Path, ...]
    shape: tuple[int, int]
    frame_count: int
    worker_count: int
    elapsed_seconds: float

    @property
    def temporary_directory(self) -> Path:
        return Path(self._temporary_directory.name)

    @property
    def frames_per_second(self) -> float:
        if self.elapsed_seconds == 0:
            return 0.0

        return self.frame_count / self.elapsed_seconds
    
    def save(self, output_path: str | Path, *, writer: Any | None = None, **writer_options: Any) -> AnimationResult:
        """
        Render the animation and save the combined output.

        The writer is inferred from the output extension unless explicitly
        provided. Rendering occurs in temporary chunk files and the requested
        output path is replaced only after successful completion.

        Parameters
        ----------
        output_path : str | Path
            Destination file path.
        writer : AnimationWriter | None, optional
            Explicit writer. If omitted, one is selected from ``WRITERS``.
        **writer_options : Any
            Options passed to the inferred writer factory.

        Returns
        -------
        SaveResult
            Output path, frame count, worker count, and elapsed render time.

        Raises
        ------
        ValueError
            If no writer is registered for the output extension.
        """

        if self._temporary_directory is None:
            raise RuntimeError("Rendered frame data is unavailable.")

        output_path = Path(output_path).expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)

        selected_writer = self._resolve_writer(output_path, writer, writer_options)

        staging_path = self._create_staging_path(output_path)
        tmp_dir = Path(self._temporary_directory.name)

        try:
            selected_writer.save_frames(self.frame_paths, staging_path, tmp_dir, self.shape)
            os.replace(staging_path, output_path)

        finally:
            staging_path.unlink(missing_ok=True)

        return AnimationResult(
            output_path,
            self.frame_count,
            self.worker_count,
            self.elapsed_seconds
        )
    
    def _resolve_writer(self, output_path: Path, writer: AnimationWriter | None, writer_options: dict[str, Any])-> AnimationWriter:
        """
        Select an explicit writer or construct one from the file extension.

        Parameters
        ----------
        output_path : Path
            Destination path used for extension inference.
        writer : AnimationWriter | None
            Explicit writer, or ``None`` to infer one.
        writer_options : dict[str, Any]
            Options passed to the inferred writer factory.

        Returns
        -------
        AnimationWriter
            Writer used for rendering and chunk combination.

        Raises
        ------
        ValueError
            If no writer is registered for the output extension.
        """
        
        if writer is not None:
            return writer

        extension = output_path.suffix.lower()
        
        if extension in WRITERS:
            return make_writer(WRITERS[extension], **writer_options)
        
        raise ValueError(
            f"No default writer for output extension {extension!r}; "
            "provide writer= explicitly"
        ) 
        
    @staticmethod
    def _create_staging_path(output_path: Path) -> Path:
        """
        Reserve a unique staging path beside the final output file.

        The temporary file created by ``mkstemp`` is immediately removed so the
        writer receives a nonexistent path while retaining a collision-safe name.

        Parameters
        ----------
        output_path : Path
            Final output path.

        Returns
        -------
        Path
            Unique nonexistent path in the output directory.
        """
        
        descriptor, staging_name = mkstemp(
            prefix=f".{output_path.stem}_",
            suffix=output_path.suffix,
            dir=output_path.parent,
        )

        os.close(descriptor)

        staging_path = Path(staging_name)

        # Writers should create their output rather than receiving an existing
        # empty file.
        staging_path.unlink()

        return staging_path

    def __str__(self) -> str:
        return (
            f"Saved {self.frame_count:,} frames to {self.temporary_directory}\n"
            f"Workers: {self.worker_count}\n"
            f"Elapsed: {self.elapsed_seconds:.2f} s\n"
            f"Render rate: {self.frames_per_second:.2f} frames/s"
        )