"""Render a precomputed propagating sine wave."""

from dataclasses import dataclass
from pathlib import Path

import matplotlib
matplotlib.use("Agg")

from typing import cast
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import numpy as np

from .animate import AnimationScene, ParallelAnimation


class parallelVideo:
    def __init__(self,odeSol):
        @dataclass(frozen=True)
        class WaveData:
            x: np.ndarray
            y: np.ndarray

        # Load/Precompute animation data.
        self.frames=np.shape(odeSol)[0] # May need a "smarter" way to pick the frame axis, since Rk45 needs a transpose
    
    def initialize_sim(self,inputFigure,inputArtists) -> AnimationScene:
        """Create the Matplotlib scene used by each worker."""
        # Inputs:
        #   inputFigure : MatPlotLib Figure object
        #   inputArtists: dictionary of static MatPlotLib artists

        # Consider moving this return statement into makeVideo to reduce function calls/shorten code
        return AnimationScene(
            figure=inputFigure,
            artists=inputArtists,
        )

    # def update_wave(self, frame: int, scene: AnimationScene) -> None:
    #     """Load the precomputed data for one frame."""
    #     wave = cast(Line2D, scene.artists["wave"])
    #     wave.set_ydata(self.DATA.y[frame])

    def makeVideo(self,inputFigure,inputArtists,inputFunc) -> None: 

        #Find a smarter way to do this than just copy pasting in the function again
        def initialize_sim(inFigure,inArtists) -> AnimationScene:
            """Create the Matplotlib scene used by each worker."""
            # Inputs:
            #   inputFigure : MatPlotLib Figure object
            #   inputArtists: dictionary of static MatPlotLib artists

            # Consider moving this return statement into makeVideo to reduce function calls/shorten code
            return AnimationScene(
                figure=inFigure,
                artists=inArtists,
            )
        animation = ParallelAnimation(
            frames=range(self.frames),
            init_func=initialize_sim(inputFigure,inputArtists),
            func=inputFunc
        )
        render = animation.render(dpi=300)
        print(render)
        save_res = render.save(Path("wave.mp4"), fps=250, ffmpeg_threads=4)
        # print(save_res)