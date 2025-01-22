NOTE: Simulation classes should not use kinematic models and instead rely on rigid body solving. 
Kinematic models rely on constraints such as wheels not slipping which may not be accurate to real world.
It is fine to use these models in control code but the control code should be verified that they are robust to these errors.