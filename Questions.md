Step 1: Use the robotic toolbox to build a PUMA robot and move the end-effector to follow a
linear trajectory along a sphere
• Use the DH table derived during the lecture
• The parameters of the PUMA robot are those in the solution of tutorial 4, and the points
of the circular trajectory will be provided.
• Useful Matlab functions: Link, SerialLink, transl, ikine, plot

--------------------------------------------------------------------------------------------

Step 2: Ensure that the end-effector is always perpendicular to the sphere surface
• The normal to the surface at each point of the circular trajectory will be provided in
terms of a vector and angle orientation for the end-effector
• Useful Matlab functions: angvec2r,rt2tr, ikine, plot

--------------------------------------------------------------------------------------------

Step 3 (advanced): Repeat steps 1 and 2 keeping a constant velocity of the end-effector of
0.1m/s for a time-varying trajectory due to the sphere changing its size (is beating!)
• The time-varying trajectory and normal to the surface at each point will be provided

