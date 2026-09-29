from dataclasses import dataclass
from typing import Any

from matplotlib.artist import Artist
from matplotlib.figure import Figure

@dataclass
class AnimationScene:
    """
    Store the figure, artists, and mutable state for an animation scene.

    Parameters
    ----------
    figure : Figure
        Matplotlib figure rendered for each frame.
    artists : dict[str, Artist]
        Named artists updated during animation.
    state : Any, optional
        Additional user-defined scene state.
    """
    figure: Figure
    artists: dict[str, Artist]
    state: Any = None
