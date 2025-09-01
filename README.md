# variable_graph_MAS
Install dependencies by running: 
    pip install -r requirements.txt
### Plug-and-Play (PnP) Controller

The system implements a **contractive PnP controller** that ensures:

1. **Individual Navigation**: Each agent follows navigation fields for obstacle avoidance
2. **Graph Maintenance**: Communication edges preserved via the Weak Invariance Principle (WIP)
3. **Stability Guarantees**: Lyapunov-based analysis ensures convergence

#### Core Control Law

For agent *i*, the control input is:

```
u_i = -γ∇φ(x_i) + Σ(j∈N_i) ω(||x_i - x_j|| - r_safe*)∇||x_i - x_j||
```

Where:
- `γ`: Leader gain parameter
- `φ(x_i)`: Navigation field at agent position
- `ω`: Cooperation gain for graph maintenance
- `r_safe*`: Safe communication radius
- `N_i`: Neighbor set of agent *i*

#### Navigation Fields

The framework supports multiple navigation field types:

- **Sphere World**: Convex obstacles with analytical gradients
- **Star-Shaped**: Non-convex domains via diffeomorphism
- **Polygonal**: Complex geometric environments

#### Theoretical Guarantees

- **(R,δ)-Goodness Condition**: Navigation fields satisfy bounded angular deviation
- **Weak Invariance Principle**: Communication graph edges remain within connectivity radius
- **Convergence**: Global asymptotic stability to formation goal
