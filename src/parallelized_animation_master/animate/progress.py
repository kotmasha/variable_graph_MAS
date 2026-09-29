import time

def format_duration(seconds: float) -> str:
    """
    Format a duration as ``MM:SS`` or ``H:MM:SS``.

    Parameters
    ----------
    seconds : float
        Duration in seconds.

    Returns
    -------
    str
        Formatted duration string.
    """
    seconds = int(seconds)
    minutes, seconds = divmod(seconds, 60)
    hours, minutes = divmod(minutes, 60)

    if hours:
        return f"{hours:d}:{minutes:02d}:{seconds:02d}"

    return f"{minutes:02d}:{seconds:02d}"


def print_status(title: str, completed: int, total_frames: int, elapsed: float) -> None:
    """
    Print the current rendering progress on a single terminal line.

    Parameters
    ----------
    completed : int
        Number of completed frames.
    total_frames : int
        Total number of frames to render.
    elapsed : float
        Elapsed rendering time in seconds.
    """
    percent = 100.0 * completed / total_frames
    rate = completed / elapsed if elapsed > 0.0 else 0.0

    print(
        f"\r{title} {completed}/{total_frames} "
        f"({percent:5.1f}%) | "
        f"elapsed {format_duration(elapsed)} | "
        f"{rate:5.2f} frames/s",
        end="",
        flush=True,
    )
    
class _LocalProgress:
    """
    Track and display rendering progress for single-process execution.

    Parameters
    ----------
    total_frames : int
        Total number of frames expected to complete.
    """
    def __init__(self, total_frames: int) -> None:
        self.total_frames = total_frames
        self.completed = 0
        self.started_at = time.perf_counter()


    def put(self, item: int) -> None:
        """
        Record completed frames and update the displayed progress.

        Parameters
        ----------
        amount : int
            Number of newly completed frames.
        """
        self.completed += item
        elapsed = time.perf_counter() - self.started_at
        print_status("Rendering frame", self.completed, self.total_frames, elapsed)
        if self.completed >= self.total_frames:
            print()
