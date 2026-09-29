# Parallel Animation

A small utility for rendering **precomputed Matplotlib animation data** in parallel.

The intended workflow is:

1. Compute or load all frame data first.
2. Create a Matplotlib scene for each worker.
3. Update that scene by indexing the data for the current frame.
4. Render frame chunks in parallel and combine them into the final output.

The animation callback should ideally perform little or no simulation work.

## Example

Suppose a propagating wave has already been computed:

```python
frames = 24

x = np.linspace(0.0, 2.0 * np.pi, 600)
phase = 2.0 * np.pi * np.arange(frames) / frames

y = np.sin(x[None, :] - phase[:, None])
```

Here,

```python
y.shape == (24, 600)
```

so each row contains all values needed to render one frame.

### Create the scene

Each worker initializes its own Matplotlib figure and artists:

```python
def initialize_wave() -> AnimationScene:
    figure, axes = plt.subplots()

    line, = axes.plot(x, y[0])

    return AnimationScene(
        figure=figure,
        artists={"wave": line},
    )
```

### Render by indexing existing data

The frame callback only selects the appropriate row:

```python
def update_wave(frame: int, scene: AnimationScene) -> None:
    wave = cast(Line2D, scene.artists["wave"])
    wave.set_ydata(DATA.y[frame])
```

This separation is intentional: the numerical work happens before rendering, while `ParallelAnimation` handles visualization and encoding.

### Save the animation

```python
animation = ParallelAnimation(
        frames=range(frames),
        init_func=initialize_wave,
        func=update_wave
    )

render = animation.render(dpi=300, worker_count=4)
print(render)

render.save(Path("wave.mp4"), fps=12)

```

Frames are divided into contiguous chunks and rendered by separate processes. Each worker reuses one `AnimationScene` for its assigned frames, and the resulting chunk files are combined into the final output.

## Requirements

* Python 3.10+
* `matplotlib`
* `ffmpeg` available on `PATH` for MP4 output


FFmpeg must be installed separately through the operating system or package manager.
