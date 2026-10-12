"""Render a precomputed propagating sine wave."""

from dataclasses import dataclass
from pathlib import Path

import matplotlib
matplotlib.use("Agg")

from typing import cast
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from multiprocessing import Process, freeze_support, set_start_method
import numpy as np
import copy

from .animate import AnimationScene, ParallelAnimation


class parallelVideo:
    def __init__(self,odeSol,figureInfo,artistDictionary,stateVecInfo,agentNames):
        @dataclass(frozen=True)
        class WaveData: # Probably remove this class
            x: np.ndarray
            y: np.ndarray

        # Load/Precompute animation data.
        self.agentNames=agentNames
        self.odeSol=odeSol
        self.frames=np.shape(odeSol)[0] # May need a "smarter" way to pick the frame axis, since Rk45 needs a transpose
        self.figureInfo=figureInfo # Not sure what datatype; probably a tuple/dictionary containing all info to correctly create the figure in THIS file.
        self.artistBook=artistDictionary # {"Name1":obj1, "Name2":obj2, etc...}. Refer to example.py/initialize_wave for data types
        self.stateVecInfo=stateVecInfo

    def initialize_sim(self) -> AnimationScene:
        """Create the Matplotlib scene used by each worker."""
        # Planned function: Take input data of OdeSol,

        # Planned outputs: artists=dictionary like {"Zoe":coord0, "Bravo":coord1, etc.}
        # Refer to example.py for the right data type.

        inputFigure,axes=plt.subplots(figsize=(10,10))
        axes.set_title("Multithreaded animation test")
        axes.set_xlim(0,10)
        axes.set_ylim(0,10)

        # for staticArtist in self.figureInfo:
        #     inputFigure.add_artist(staticArtist)

        inputArtists={}
        for title in self.artistBook:
            inputArtists[title]=patches.Circle(
            tuple(self.artistBook[title]),
            radius=0.15,
            label=title,
            color='orange',
            animated=True,
            )

        return AnimationScene(
            figure=inputFigure,
            artists=inputArtists,
        )
    
    def update_func(self,frame: int, scene: AnimationScene) -> None:
        """Load the precomputed data for one frame."""
        for agent in self.agentNames:
            dot=cast(patches.Circle, scene.artists[agent])
            dot.set(center=tuple(self.odeSol[frame,self.stateVecInfo[agent][0]:self.stateVecInfo[agent][1]]))

    # def update_wave(self, frame: int, scene: AnimationScene) -> None:
    #     """Load the precomputed data for one frame."""
    #     wave = cast(Line2D, scene.artists["wave"])
    #     wave.set_ydata(self.DATA.y[frame])

    def makeVideo(self) -> None: 
        animation = ParallelAnimation(
            frames=range(self.frames),
            init_func=self.initialize_sim,
            func=self.update_func
        )
        render = animation.render(dpi=300)
        print(render)
        save_res = render.save(Path("pnpMovie.mp4"), fps=60, ffmpeg_threads=4)
        # print(save_res)