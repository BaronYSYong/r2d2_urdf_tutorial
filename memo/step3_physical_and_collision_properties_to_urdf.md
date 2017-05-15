# Step 3: Adding Physical and Collision Properties to a URDF Model

## 1. Collision
In order to get collision detection to work or to simulate the robot in something like Gazebo, we need to define a collision element as well. 
Here is the code for new base link.
```
<link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 .8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
  </link>
```
* The collision element is a direct subelement of the link object, at the same level as the visual tag
* The collision element defines its shape the same way the visual element does, with a geometry tag. The format for the geometry tag is exactly the same here as with the visual.
* You can also specify an origin in the same way as a subelement of the collision tag (as with the visual)

In many cases, you’ll want the collision geometry and origin to be exactly the same as the visual geometry and origin. However, there are two main cases where you wouldn’t.

* Quicker Processing - Doing collision detection for two meshes is a lot more computational complex than for two simple geometries. Hence, you may want to replace the meshes with simpler geometries in the collision element.

* Safe Zones - You may want to restrict movement close to sensitive equipment. For instance, if we didn’t want anything to collide with R2D2’s head, we might define the collision geometry to be a cylinder encasing his head to prevent anything from getting to near his head.

## 2. Physical Properties
In order to get your model to simulate properly, you need to define several physical properties of your robot, i.e. the properties that a physics engine like Gazebo would need.

### 2.1 Inertia
Every link element being simulated needs an inertial tag. Here is a simple one.
```
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 .8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="0.4" ixy="0.0" ixz="0.0" iyy="0.4" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>
```
This element is also a subelement of the link object.
The mass is defined in kilograms.
The 3x3 rotational inertia matrix is specified with the inertia element. Since this is symmetrical, it can be represented by only 6 elements, as such.


| ixx | ixy | ixz |
|-----|-----|-----|
| ixy | iyy | iyz |
| ixz | iyz | izz |

* This information can be provided to you by modeling programs such as MeshLab. The inertia of geometric primitives (cylinder, box, sphere) can be computed using Wikipedia's list of moment of inertia tensors (and is used in the above example).
    * https://en.wikipedia.org/wiki/List_of_moments_of_inertia
* The inertia tensor depends on both the mass and the distribution of mass of the object. A good first approximation is to assume equal distribution of mass in the volume of the object and compute the inertia tensor based on the object's shape, as outlined above.
* If unsure what to put, a matrix with ixx/iyy/izz=1e-3 or smaller is often a reasonable default for a mid-sized link (it corresponds to a box of 0.1 m side length with a mass of 0.6 kg). Although often chosen, the identity matrix is a particularly bad default, since it is often much too high (it corresponds to a box of 0.1 m side length with a mass of 600 kg!).
* You can also specify an origin tag to specify the center of gravity and the inertial reference frame (relative to the link's reference frame).
* When using realtime controllers, inertia elements of zero (or almost zero) can cause the robot model to collapse without warning, and all links will appear with their origins coinciding with the world origin.

### 2.2 Contact Coefficients
You can also define how the links behave when they are in contact with one another. This is done with a subelement of the collision tag called contact_coefficients. There are three attributes to specify:

* mu - The friction coefficient
* kp - Stiffness coefficient
* kd - Dampening coefficient

### 2.3 Joint Dynamics
How the joint moves is defined by the dynamics tag for the joint. There are two attributes here:

* friction - The physical static friction. For prismatic joints, the units are Newtons. For revolving joints, the units are Newton meters.
* damping - The physical damping value. For prismatic joints, the units are Newton seconds per meter. For revolving joints, Newton meter secons per radian.

If not specified, these coefficients default to zero.

## 3. Reference
* In the realm of pure URDF (i.e. excluding Gazebo-specific tags), there are two remaining tags to help define the joints: calibration and safety controller. 
    * http://wiki.ros.org/urdf/XML/joint