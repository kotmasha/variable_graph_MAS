# Plug-and-Play Cooperative Navigation Framework

Implementation of the PnP cooperative navigation framework from "Plug-and-Play Cooperative Navigation: From Single-Agent Navigation Fields to Graph-Maintaining Distributed MAS Controllers" (IEEE Transactions on Automatic Control, 2024).

## Overview

This framework enables distributed multi-agent navigation while maintaining communication connectivity in obstacle environments. The core idea: take any single-agent navigation method and automatically extend it to multi-agent coordination using closed-form controllers.

Key capabilities:
- Multi-agent coordination with distance-limited communication
- Communication graph maintenance
- Obstacle avoidance in complex environments
- Works with any single-agent navigation field
- Real-time performance
- Trajectory and connectivity visualization

## Theory

**Paper**: "Plug-and-Play Cooperative Navigation: From Single-Agent Navigation Fields to Graph-Maintaining Distributed MAS Controllers"  
**Authors**: Dan P. Guralnik, Peter F. Stiller, Federico M. Zegers, Warren E. Dixon  
**Published**: IEEE Transactions on Automatic Control, Vol. 69, No. 8, August 2024  
**DOI**: [10.1109/TAC.2023.3346796](https://doi.org/10.1109/TAC.2023.3346796)

The PnP controller for agent p:
```
ẋⵗₘ = Σ ξⵗₘᵩ n(xᵩ, xⵗₘ) + vⵗₘ
     q~p
```

- `n(y,z)`: navigation field providing obstacle-free paths
- `ξⵗₘᵩ`: state-dependent coupling gains
- `vⵗₘ`: task component (leader tracking, formation, etc.)

## Structure

```
src/           # Core implementation
config/        # YAML configuration files  
docs/          # Documentation and paper
examples/      # Usage examples
results/       # Simulation outputs
```

## Usage

**Install dependencies:**
```bash
pip install numpy matplotlib scipy shapely qpsolvers descartes pyyaml
```

**Run simulation:**
```bash
cd src/
python main_single_sim.py ../config/easyTest.yml 10
```
Arguments: configuration file, simulation time (seconds)

**Environment types:**
- `worldType: 0` - Circular obstacles
- `worldType: 1` - Star-shaped obstacles  
- `worldType: 2` - Rectangular obstacles

**Controller types:**
- `Lazy: 0` - Contractive (continuous attraction)
- `Lazy: 1` - Lazy (minimal control effort)

## Configuration

See `config/` directory for examples. Key parameters:

```yaml
pnpParameters:
  Lazy: 0|1            # Controller type
  coopGain: float      # Cooperation strength  
  leaderGain: float    # Leader tracking gain
  rsafe: float         # Safe distance
  rcomm: float         # Communication range

Agents:
  Names: [...]         # Agent identifiers
  Leaders:             # Leader specifications
    AgentName:
      Target: [x, y]   # Goal position
```


## References

Guralnik, D.P., et al. "Plug-and-Play Cooperative Navigation: From Single-Agent Navigation Fields to Graph-Maintaining Distributed MAS Controllers." IEEE Transactions on Automatic Control, 2024.

## License

MIT License - see [LICENSE](LICENSE) file.

Funding: AFOSR Awards FA9550-19-1-0169, FA9550-18-1-0109; ONR Grant N00014-21-1-2481
