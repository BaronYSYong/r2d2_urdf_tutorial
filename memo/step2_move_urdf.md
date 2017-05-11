# Step 2: Building a Movable Robot Model with URDF

Create a file called “06-flexible.urdf” inside urdf directory. 
To visualize and control this model, run the same command as the last tutorial: roslaunch urdf_tutorial display.launch model:=urdf/06-flexible.urdf However now this will also pop up a GUI that allows you to control the values of all the non-fixed joints. Play with the model some and see how it moves. Then, we can take a look at how we accomplished this. 