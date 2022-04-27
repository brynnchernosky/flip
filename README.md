# Fluid Simulation Using Implicit Particles

## References  
- <a href="http://www.danenglesson.com/images/portfolio/FLIP/rapport.pdf"> Fluid Simulation Using Implicit Particles </a>
- <a href="https://cg.informatik.uni-freiburg.de/intern/seminar/gridFluids_fluid_flow_for_the_rest_of_us.pdf"> Fluid Flow for the Rest of Us: Tutorial of the Marker and Cell Method in Computer Graphics </a>

## Overview
- In MacGrid::init(), initial fluid position is defined based on input trimesh 
- Within the MacGrid::simulate() loop:
    - Timestep is calculated based on the CFL condition
    - External forces (gravity) are applied to the grid cells
    - Dirichlet boundary condition is enforced (velocity field into solids is 0)
    - A buffer zone is created surrounding the fluid
    - Divergence is removed from the velocity field 
    - Particle positions are updated based on velocity field
    - Occassionally, particle positions are output for conversion into output trimesh
