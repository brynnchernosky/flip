# Fluid Simulation Using Implicit Particles

## References  
- <a href="http://www.danenglesson.com/images/portfolio/FLIP/rapport.pdf"> Fluid Simulation Using Implicit Particles </a>
- <a href="https://cg.informatik.uni-freiburg.de/intern/seminar/gridFluids_fluid_flow_for_the_rest_of_us.pdf"> Fluid Flow for the Rest of Us: Tutorial of the Marker and Cell Method in Computer Graphics </a>

## Main Files
- Main.cpp
- MacGrid.cpp
- Reconstruction.cpp

## Overview
- Initial fluid/solid positions are defined based on input trimeshes 
- Within the simulation loop:
    - A buffer zone is created surrounding the fluid
    - Velocity field updated based on particle velocities
    - Velocity field updated based on external forces (gravity) 
    - Velocity field into and out of solids is set to 0
    - Divergence is removed from the velocity field
    - Particle positions are updated based on velocity field
    - Particle positions are output 
- Marching cubes is used to produce a trimesh, which is then rendered

## Extensions
- Adding additional fluid to simulation by adding fluid source or turning solids into fluids
- Adding foam particles to simulation when cell curl above given value 
