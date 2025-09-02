# Variable Graph Multi-Agent System (MAS) Controller

A Python implementation of the **Proximity-based Navigation Protocol (PnP)** for multi-agent coordination and navigation in obstacle-rich environments. This framework enables distributed control of agent networks with dynamic graph topologies and obstacle avoidance capabilities.

## Overview

This repository implements a distributed multi-agent system where agents coordinate their movements using proximity-based protocols while navigating around obstacles. The system supports:

- **Graph-based agent networks** with configurable topologies
- **Obstacle avoidance** in sphere worlds and polygonal environments  
- **Leader-follower coordination** with target tracking
- **Real-time visualization** and animation export
- **Configurable PnP parameters** for different coordination behaviors

## Dependencies

This Python package requires several scientific computing and optimization libraries. Install all dependencies by running:

```bash
pip install -r requirements.txt
```

### Core Dependencies:
- `qpsolvers[open_source_solvers]` - Quadratic programming solvers for navigation functions
- `quadprog` - Additional QP solver
- `ffmpeg` - Video encoding for animation export
- `numpy` - Numerical computing
- `scipy` - Scientific computing and optimization
- `matplotlib` - Visualization and animation
- `shapely` - Geometric operations
- `pyyaml` - Configuration file parsing

### Optional Dependencies:
- `rclpy` - ROS2 integration (for distributed deployment)
- `geometry_msgs` - ROS2 message types
- `tf_transformations` - Coordinate transformations

## Quick Start

Run a basic simulation with the provided example configuration:

```bash
python src/main_single_sim.py src/easyTest.yml 100
```

Where:
- `src/easyTest.yml` - Configuration file defining the scenario
- `100` - Simulation time in seconds

## Configuration

The system is configured through YAML files that specify:

### Workspace Parameters
```yaml
Wksp_bnds:
  upper: [[15]]    # Upper bounds of workspace
  lower: [[0]]     # Lower bounds of workspace
```

### Agent Network Definition
```yaml
Agents:
  stateWnames: 1   # 1 if providing initial positions, 0 for random spawn
  NamesandPos:
    - ["Joe", [4,5]]   # Agent name and initial position
    - ["Josh", [2,4]]
    - ["Bob", [4,4]]
  tree_mat_names:
    - ["Bob","Joe"]    # Network edges (communication links)
    - ["Joe","Josh"]
  Leaders:
    Joe:
      Target: [14,14]   # Leader target destination
      keepUpQ: False    # Whether leader maintains formation
```

### PnP Parameters
```yaml
pnpParameters:
  Lazy: 1          # Enable lazy cooperation mode
  coopGain: 10     # Cooperation strength
  leaderGain: 0.01 # Leader tracking gain (gamma)
  alpha: 1.01      # Tension function exponent
  rsafe: 0.7       # Safety radius
  rcomm: 3         # Communication radius
```

### Environment Setup
```yaml
Obstacles:
  worldType: 0     # 0=sphere world, 1=star-shaped, 2=polygonal
  sphereWorld:
    obsCenter:
      - [3,12]       # Obstacle centers
      - [12,3]
      - [7,7]
    obsRadii: [2,2,3] # Obstacle radii
```

## Architecture

### Core Components

**A) Environment Module (`Environment.py`)**
- Manages workspace boundaries and obstacle definitions
- Implements navigation functions using quadratic programming
- Supports multiple environment types:
  - `sphereworldEnv`: Circular obstacles
  - `polygonEnv`: Polygonal obstacles  
  - `starWorldEnv`: Star-convex obstacles

**B) Agent Module (`agent.py`)**
- Defines base agent class with PnP coordination logic
- Handles neighbor communication and position polling
- Implements proximity-based control inputs
- Supports ROS2 integration for distributed deployment

**C) Network Module (`Network.py`)**
- Manages the multi-agent network structure
- Coordinates agent spawning and initialization
- Implements network-wide PnP updates and visualization
- Handles tension functions and cooperation parameters

**D) Graph Module (`graph_w_names.py`)**
- Represents agent communication topology
- Provides neighbor queries and graph traversal
- Supports depth-first search for agent initialization

**E) Task Assignment (`agentTask.py`)**
- Assigns individual tasks to agents
- Distinguishes between leaders (with targets) and followers
- Manages formation maintenance behaviors

## Simulation Modes

The system supports three integration methods:

1. **Animation Mode** (default): Real-time visualization with frame-by-frame updates
2. **Euler Integration**: Fixed-step numerical integration
3. **ODE Integration**: Adaptive step-size integration using `scipy.integrate.odeint`

Set the mode in your YAML configuration:
```yaml
solverType: 'animation'  # or 'Euler' or 'odeInt'
```

## Output

### Visualization
- Real-time matplotlib animation showing:
  - Agent positions (purple=leaders, orange=followers)
  - Communication links between connected agents
  - Workspace boundaries and obstacles
  - Leader target destinations

### Exported Files
- `pnpMovie.mp4`: Animation video of the simulation
- `runData.json`: Simulation data (when applicable)
- Various plot files in `output_plots/` directory

## Example Scenarios

Several pre-configured scenarios are provided:

- `easyTest.yml`: Simple 3-agent network in sphere world
- `randomSpawnNoObs.yml`: Random initialization without obstacles
- `edgeCases.yml`: Challenging configurations for testing
- `agentsWithObsnPos.yml`: Complex multi-obstacle scenarios

## Algorithm Details

### Proximity-based Navigation Protocol (PnP)
The PnP algorithm enables agents to:
1. **Maintain formation** through proximity-based cooperation
2. **Avoid obstacles** using safe navigation functions
3. **Track targets** when designated as leaders
4. **Adapt network topology** based on communication constraints

### Key Features
- **Tension Functions**: Distance-dependent cooperation that varies smoothly between safety and communication radii
- **Lazy Cooperation**: Optional mode that reduces control effort while maintaining connectivity
- **Safety Guarantees**: Obstacle avoidance through convex optimization
- **Scalable Networks**: Supports arbitrary graph topologies and agent numbers

## Usage Examples

### Basic Simulation
```bash
# Run 3-agent formation with obstacles for 50 seconds
python src/main_single_sim.py src/easyTest.yml 50
```

### Custom Configuration
1. Copy an existing YAML file
2. Modify agent positions, network topology, or obstacles
3. Adjust PnP parameters for desired behavior
4. Run simulation with your custom configuration

### Batch Processing
```bash
# Multiple simulation runs with different parameters
python src/main_single_sim.py src/scenario1.yml 100
python src/main_single_sim.py src/scenario2.yml 100
```

## Research Context

This implementation is based on research in distributed multi-agent coordination and formation control. The PnP framework provides theoretical guarantees for:
- Network connectivity maintenance
- Obstacle avoidance safety
- Target convergence for leader agents
- Scalability to large agent networks

## Troubleshooting

### Common Issues
1. **ROS2 Dependencies**: If you encounter `rclpy` import errors, the simulation can run without ROS2 by commenting out ROS-related imports
2. **Animation Path**: The video output path may need adjustment for your operating system
3. **Solver Convergence**: Increase `mxstep` parameter in ODE integration if encountering convergence issues

### Performance Tips
- Use `--single-branch` when cloning for faster download
- Reduce animation frames (`Nframes`) for faster processing
- Use Euler integration for real-time applications

## Contributing

This project is part of ongoing research in multi-agent systems. Contributions are welcome, particularly:
- Additional environment types
- New agent behaviors
- Performance optimizations
- Documentation improvements

## License

MIT License - see `LICENSE` file for details.

## Citation

If you use this code in your research, please cite the relevant publications on PnP multi-agent navigation protocols.
