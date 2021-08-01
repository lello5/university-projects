Basic 26 points features: done (I've also rendered the highres images).

Extra credit (every result is contained here inside folder "out"):

- Large Scenes (2 points): I've rendered the large scenes.

- Refraction (2 points): I've added code for refraction (in sample and eval functions). It works for thick glass (i.e. lamps in bedroom) and its behavior isn't showing in 05_glass because the glass is thin there.

- MYOS (2 points): based on the test scene, I've added an HDR environment map for the background; added a texture to the floor; moved, oriented and scaled old and new models (tree, guitar, car, mug) using the .json file; added "a lot" of textures and material behaviors to the objects. 
The final implemented scene aims to be as Christmassy as possible :D

- Stratified Sampling 1 (4 points): I've implemented a new shader "shade_stratified" that only sends rays out uniformly in the cosine-weighted hemisphere and consequently made some minor changes in sample_camera to adapt to it and generate the stratified sampling. In folder "extracredit_stratified_sampling" I'm including the 02_matte images produced by naive, path and stratified shader for comparison (both high and low resolution): looking at them we can easily see how lines are much more emphasized and visible in the stratified images, even if on the contrary they appear as dashed in some points. For comparison, I'm also adding 2 zoomed images.