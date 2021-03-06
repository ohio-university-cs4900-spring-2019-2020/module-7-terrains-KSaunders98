# Module 7: Terrains + Physics
This module demonstrates loading a real world terrain into the game engine along with attaching a physics actor to it. The physics are networked between two instances of the game engine. One instance performs the physics simulation, and the other reflects the results.

The terrain loaded in this demo is from Gallipolis, Ohio, right on the Ohio River. See aftr.conf for the specific coordinates of the terrain sample.

***NOTE:*** This repo makes use of Git LFS (large file storage) for the actual terrain data located in *mm/images*. In order to clone the repository and retrieve this data correctly, Git LFS must be installed. See https://git-lfs.github.com/ for details.
## Instructions
- First, run one instance of the module with NetServerListenPort=12683 in the aftr.conf file and then run another instance with NetServerListenPort=12682 in the aftr.conf file.
- In either instance, ctrl+click on any area of the terrain to spawn a teapot above the point clicked.
- All physics in the server instance will be networked to the client instance.
- For best results, close the server instance before closing the client instance.