Test simulations for the custom GH-JRF path constraint in Moco

Idea of the test is to simualte an abduction motion and calculate the GH JRF
firstly without the custom path constraint - and examine the results. The hope
here is that the JRFs will violate the constraint function included in the plugin.

Following this, re-simulate the abduction motion with the path constraint included.
Hopefully the constraints from the function will no longer be violated...