from .mp4_writer import Mp4Writer
from .pdf_writer import PdfWriter
from .gif_writer import GifWriter
from .base_writer import AnimationWriter

from dataclasses import fields
from typing import Any, TypeVar, cast

WRITERS: dict[str, type[AnimationWriter]] = {
    '.mp4' : Mp4Writer,
    '.pdf': PdfWriter,
    '.gif': GifWriter
}

W = TypeVar("W", bound=AnimationWriter)

def make_writer(writer_cls: type[W], **kwargs: Any) -> W:
    """
    Construct a writer using only supported dataclass fields.

    Extra keyword arguments are ignored, allowing shared writer options to be
    passed without requiring every writer type to accept every option.

    Parameters
    ----------
    writer_cls : type[W]
        Animation writer class to instantiate.
    **kwargs : Any
        Candidate constructor arguments.

    Returns
    -------
    W
        Instantiated writer of type ``writer_cls``.
    """
    accepted = {
        field.name
        for field in fields(cast(Any, writer_cls))
    }

    return writer_cls(**{
        key: value
        for key, value in kwargs.items()
        if key in accepted
    })
