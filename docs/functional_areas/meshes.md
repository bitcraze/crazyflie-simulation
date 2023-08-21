---
title: Meshes
page_id: meshes
sort_order: 1
---

The model of the Crazyflie has been made in Blender 3.0 in such a way that has as little polygons as possible.

These files can be find in the `meshes/` folder:
*  `/meshes/blender_flies/` *.blend Blender 3.0 source files
*  `/meshes/collada_files/` *.dea Collada mesh exports of assembly with seperate props
*  `/meshes/stl_files/` *.stl STL mesh exports of the individual partts, the CF2 body and the full assembly
* `/meshes/textures/` Any additional texture files necessary for simulation.

So the mesh files assembled looks a bit like this as a render in blender:

![crazyflie render](/docs/images/cf2_render.png)


## Measurements Crazyflie Assembly

If you'd like to use the mesh files to assemble a full crazyflie, these are the location the individual parts in meters:

* Propeller origin from center (0.031, 0.031, 0.022) m
    * Mirrored in the x and y axis
* Motor mount + motor mounts (0.031, 0.031, 0.014) m
    * Mirrored in the x and y axis
* Body origin height from center ( 0, 0, 0.015) m
* Battery holder height ( 0, 0,0.025) m
* Battery height (0,0,0.02) m
* Pin headers (0, 0.011, 0.014) m
    * Mirrored in the y axis

