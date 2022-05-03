# Fluid Simulation Using Implicit Particles

## References  
- <a href="http://www.danenglesson.com/images/portfolio/FLIP/rapport.pdf"> Fluid Simulation Using Implicit Particles </a>
- <a href="https://cg.informatik.uni-freiburg.de/intern/seminar/gridFluids_fluid_flow_for_the_rest_of_us.pdf"> Fluid Flow for the Rest of Us: Tutorial of the Marker and Cell Method in Computer Graphics </a>

## Overview
- In MacGrid::init(), initial fluid position is defined based on input trimesh 
- Within the MacGrid::simulate() loop:
    - A buffer zone is created surrounding the fluid
    - Velocity field updated based on particle velocities
    - External forces (gravity) applied to the grid cells
    - Fluid velocity field extrapolated to air cells
    - Dirichlet boundary condition enforced (velocity field into solids is 0)
    - Divergence removed from the velocity field
    - Fluid velocity field extrapolated to air cells
    - Particle positions are updated based on velocity field
    - Particle positions output for conversion into output trimesh
- Extensions:
    - Adding additional fluid to simulation
    - Adding foam particles to simulation when cell curl above given value 
