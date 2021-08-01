# 2021-homework1
Template repository for Homework1 of Interactive Graphics 2021

This repository contains the initial files for the first homework of the Interactive Graphics course of the Master course in Artificial Intelligence and Robotics (and also of the Engineering in Computer Science course) of Sapienza University of Rome. The assignment in PDF format is in the repository, the textual format follows here:

You need to modify the files so to obtain the following effects: 

1.	Replace the cube with a more complex and irregular geometry of 20 to 30 (maximum) vertices. Each vertex should have associated a normal (3 or 4 coordinates) and a texture coordinate (2 coordinates). Explain in the document how you chose the normal and texture coordinates.
2.	Compute the barycenter of your geometry and include the rotation of the object around the barycenter and along all three axes. Control with buttons/menus the axis and rotation, the direction and the start/stop.
3.	Add the viewer position (your choice), a perspective projection (your choice of parameters) and compute the ModelView and Projection matrices in the Javascript application. The viewer position and viewing volume should be controllable with buttons, sliders or menus. Please choose the initial parameters so that the object is clearly visible and the object is completely contained in the viewing volume. By changing the parameters you should be able to obtain situations where the object is partly or completely outside of the view volume. 
4.	Add a cylindrical neon light, model it with 3 light sources inside of it and emissive properties of the cylinder. The cylinder is approximated by triangles. Assign to each light source all the necessary parameters (your choice). The neon light should also be inside the viewing volume with the initial parameters. Add a button that turns the light on and off.
5.	Assign to the object a material with the relevant properties (your choice).
6.	Implement both per-vertex and per-fragment shading models. Use a button to switch between them.
7.	Create a procedural normal map that gives the appearance of a very rough surface. Attach the bump texture to the geometry you defined in point 1. Add a button that activates/deactivates the texture.

Describe your solution in a short PDF document (2-3 pages) describing the techniques used, the advantages and disadvantages of the proposed solution and the features of your solution. 

How to submit the homework
The solution should be delivered on the GitHub Classroom repository. DO NOT USE YOUR GITHUB PERSONAL ACCOUNT. Do not post solutions or code on Google Classroom. Use Google Classroom only for questions and clarifications. Do not ask for clarifications or comments by email, use only Google Classroom
