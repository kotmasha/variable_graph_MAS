"""Render a precomputed propagating sine wave."""

from dataclasses import dataclass
from pathlib import Path

import matplotlib
matplotlib.use("Agg")

from typing import cast
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import numpy as np

from animate import AnimationScene, ParallelAnimation


@dataclass(frozen=True)
class WaveData:
    x: np.ndarray
    y: np.ndarray


# Precompute all animation data.
frames = 500
x = np.linspace(0.0, 2.0 * np.pi, 600)
phase = 2.0 * np.pi * np.arange(frames) / frames

DATA = WaveData(
    x=x,
    y=np.sin(x[None, :] - phase[:, None]),
)


def initialize_wave() -> AnimationScene:
    """Create the Matplotlib scene used by each worker."""
    figure, axes = plt.subplots(figsize=(7, 4))

    axes.set(
        xlim=(0.0, 2.0 * np.pi),
        ylim=(-1.2, 1.2),
        xlabel="x",
        ylabel="amplitude",
        title="Propagating sine wave",
    )

    line, = axes.plot(DATA.x, DATA.y[0])

    return AnimationScene(
        figure=figure,
        artists={"wave": line},
    )

def update_wave(frame: int, scene: AnimationScene) -> None:
    """Load the precomputed data for one frame."""
    wave = cast(Line2D, scene.artists["wave"])
    wave.set_ydata(DATA.y[frame])

def main() -> None:
    animation = ParallelAnimation(
        frames=range(frames),
        init_func=initialize_wave,
        func=update_wave
    )
    render = animation.render(dpi=300)
    print(render)
    save_res = render.save(Path("wave.mp4"), fps=250, ffmpeg_threads=4)
    # print(save_res)


if __name__ == "__main__":
    main()
