#
This project involves designing a controller that provides time-based gaurantees while completing a task, besides gauranteeing invariance of the system to never leave its safe set of states. Signal temporal logic(STL) has been the primary concept used to come up with such a controller.
Major challenge being the application chosen, that is highway driving which is an uncontrolled and dynamic environment. Another challenge was identifying and accepting the limitations of my method, which is also a lesson for life :)

The Signal Temporal Logic(STL) constraints are defined in the function files.
These are then represented as time-varying barrier functions which are used by a QP optimisation based controller to calculate the inputs to ensure that the ego vehicle never leaves the defined safe states.


# Lane Changing
 - Run the mainLane6_15.m file.



# Overtaking and Merging With Platoon
 - Run the over6_11.m file.  (Takes approx 230s to run)


#
In order to view plots of internal states and other variables, corresponding section from the main files can be uncommented.