# Step 4: Using Xacro to Clean Up a URDF File

## 1. Using Xacro
As its name implies, xacro is a macro language. The xacro program runs all of the macros and outputs the result. Typical usage looks something like this:
```
$ rosrun xacro xacro model.xacro > model.urdf 
```
You can also automatically generate the urdf in a launch file. This is convenient because it stays up to date and doesn’t use up hard drive space. However, it does take time to generate, so be aware that your launch file might take longer to start up. 
* http://wiki.ros.org/xacro
* http://wiki.ros.org/pr2_description
```
<param name="robot_description"
  command="$(find xacro)/xacro '$(find pr2_description)/robots/pr2.urdf.xacro'" />
```
At the top of the URDF file, you must specify a namespace in order for the file to parse properly. For example, these are the first two lines of a valid xacro file:
```
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="firefighter">
```

## 2. Constants
Let’s take a quick look at our base_link in R2D2.
```
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
  </link>
```
The information here is a little redundant. We specify the length and radius of the cylinder twice. Worse, if we want to change that, we need to do so in two different places.

Fortunately, xacro allows you to specify properties which act as constants. Instead, of the above code, we can write this.
```
<xacro:property name="width" value="0.2" />
<xacro:property name="bodylen" value="0.6" />
<link name="base_link">
    <visual>
        <geometry>
            <cylinder radius="${width}" length="${bodylen}"/>
        </geometry>
        <material name="blue"/>
    </visual>
    <collision>
        <geometry>
            <cylinder radius="${width}" length="${bodylen}"/>
        </geometry>
    </collision>
</link>
```
* The two values are specified in the first two lines. They can be defined just about anywhere (assuming valid XML), at any level, before or after they are used. Usually they go at the top.
* Instead of specifying the actual radius in the geometry element, we use a dollar sign and curly brackets to signify the value.
* This code will generate the same code shown above.

The value of the contents of the ${} construct are then used to replace the ${}. This means you can combine it with other text in the attribute.