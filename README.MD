# Simulating the impact of glenohumeral capsulorrhapjhy on movement kinematics and muscle function in activities of daily living.

The dataset and code provided with this read me allow replication of the model building and analysis associated with our paper:

*** TODO: add paper citation details ***

Please cite the above-mentioned paper if using any data or code (adapted or otherwise) from this dataset/repository.

The authors of this work (Aaron Fox, Jason Bonacci, Stephen Gill and Richard Page) retain ownership and copyright over this data and code.

The following sections outline relevant details regarding the folders contained within this repository.

## Code

The code folder contains two subfolders - 'Main' and 'Supplementary.' The main code snippets should be run from their own directory to ensure appropriate navigation through the different files. As the results of a number of the experiments are already obtained, certain aspects of the code are commented out to avoid repeating these analyses. These can be uncommented to run the analsis for yourself. Note that the 'ligament' optimisations do take some time due to repeatedly running through the range of motion simulations to obtain the best parameters. The movement simulations conducted through OpenSim Moco also take a ***significant*** amount of time, and the time taken should be considered when re-implementing the code (you're not going to be able to run this in a day or two...).

The main directory includes the required code and analysis steps to reproduce our models and results:

- **ShoulderCapsulorrhaphySims_1_AddModelLigaments**: This code adds the necessary 'ligaments' (i.e. model objects that produce passive resistive torque) and actuators to the basic shoulder complex model for further optimisation in step 2.
- **ShoulderCapsulorrhaphySims_2_OptimiseLigamentParameters**: This code optimises the parameters of the 'ligaments' in the model so that the peak joint angles reached during passively generated shoulder motions reach that of those specified in each of the capsulorrhaphy models. This is currently set to achieve the mean angles achieved by the cadaveric experiments performed by Gerber et al. (2003); but could be easily edited to achieve varying peak angles.
- **ShoulderCapsulorrhaphySims_3_CreateModels**: This code takes the parameters identified in step 2 and creates the iterations of the baseline model that represent the different capsulorrhaphy conditions assessed by Gerber et al. (2003). The code also creates figures and tables representing the different model data. The OpenSim analyses steps in this code are currently commented out as these results are already available in the relevant directories.
- **ShoulderCapsulorrhaphySims_4_NodeSelection**: This code identifies an appropriate mesh interval to run the different movement task simulations over. It replicates the grid refinement approach suggested by Lee & Umberger (2016). As noted above, these sorts of repeated simulations take a decent amount of computational time.
- **ShoulderCapsulorrhaphySims_5_RunSimulations**: This code can be used to run the different movement task simulations through OpenSim Moco. ***Please note that these simulations take a significant amount of computational time.*** For how to set-up OpenSim Moco in Matlab, users are referred to the [Moco SimTK project home](https://simtk.org/projects/opensim-moco). These simulations were run on Moco version 0.3.0, so some small code edits may be required to run on earlier or later versions. 
- **ShoulderCapsulorrhaphySims_6_AnalyseSimulations**: This code loads in the predictive simulations from step 5 and runs a series of analyses to extract and present the results. The OpenSim analyses steps in this code are currently commented out as these results are already available in the relevant directories. The creation of figures in this code are also commented out to avoid creating every time this code is run, so these will need to be uncommented to reproduce the visuals.

The supplementary directory includes convenience/accessory functions to assist with the main code. This includes various OpenSim based functions to alter model components or add relevant aspects into the Moco simulations. It also includes the [legendflex](https://au.mathworks.com/matlabcentral/fileexchange/31092-legendflex-m-a-more-flexible-customizable-legend) Matlab function which helps create more flexible figure legends. Please cite this function appropriately if using.

## ModelFiles

This directory includes the relevant upper limb OpenSim models used in the predictive simulations. It contains the various glenohumeral capsulorrhaphy models created in the step 3 of code outlined above. The models kinematic foundations are based on those of Holzbaur et al. (2005) and Saul et al. (2015), with the kinematics of the shoulder girdle, elbow and forearm described as recommended by Wu et al. (2005). Scapula motion is dictated by the equations outlined by de Groot & Brand (2001). The muscles that actuate the shoulder girdle are based off those described in Wu et al. (2016). Note that in the code to run the simulations the muscle model parameters are edited to reflect those presented by De Groote et al. (2016) and tendon dynamics are ignored. These settings are better suited to the direct collocation method used in OpenSim Moco.

## Node Selection

The data contained in this folder relates to step 4 of the main code outlined above. It contains the simulation results from the different movements for each of the tested nodes. It also contains visuals plotting the different outputs used to assess the final grid density used in the simulations.

## Plugin_DualEBCF

This directory contains the custom built force class plugin that is used with the OpenSim models. This force class builds on that of the ExpressionBasedCoordinateForce by allowing two coordinates to be used in the force expression. This was necessary as it was clear in the Gerber et al. (2003) data that passive resistance at the shoulder was modulated by multi-planar motion. The underlying build code as well as the output libraries are included in this folder. The main library that is used within the main code is located at \build\Release\osimDualEBCF.dll.

## Simulation Results

This folder contains the predictive simulation outputs generated through OpenSim Moco for the movements analysed, and the results from the subsequent OpenSim analyses run on these outputs. Figures comparing the outputs from the different capsulorrhaphy models are also included as sub-folders. Note that these results relate to those presented in the above-mentioned paper. Were the predictive simulations to be re-run, slight differences in the data may occur due to potential differences in the optimiser result found.

## Supporting Data

This folder contains a couple of .mat databases that are used in the main code. Specifically, one that has the results extracted from the Gerber et al. (2003) cadaveric simulations; and another that contains the typical movement ranges from Vidt et al. (2014, 2016) that are used as bounds to limit motion to expected values for the movement tasks analysed.

## Videos

This folder contains videos of the three simulated movements assessed. These were created from the baseline model (i.e. no capsulorrhaphy) data. These videos are in .webm format as output by OpenSim, and can be vieweed using VLC Media Player.

## References

de Groot JH, Brand R. 2001. A three-dimensional regression model of the shoulder rhythm. Clin Biomech, 16(9):735–743.

De Groote F, Kinney AL, Rao AV, Fregly BJ. 2016. Evaluation of Direct Collocation Optimal Control Problem Formulations for Solving the Muscle Redundancy Problem. Ann Biomed Eng, 44(10):2922–2936.

Gerber C, Werner CML, Macy JC, et al. 2003. Effect of selective capsulorrhaphy on the passive range of motion of the glenohumeral joint. J Bone Joint Surg, 85(1):48–55.

Holzbaur KRS, Murray WM, Delp SL. 2005. A Model of the Upper Extremity for Simulating Musculoskeletal Surgery and Analyzing Neuromuscular Control. Ann Biomed Eng, 33(6):829–840.

Lee LF, Umberger BR. 2016. Generating optimal control simulations of musculoskeletal movement using OpenSim and MATLAB. Peer J, 4:e1638.

Saul KR, Hu X, Goehler CM, et al. 2015. Benchmarking of dynamic simulation predictions in two software platforms using an upper limb musculoskeletal model. Comput Meth Biomech Biomed Eng, 18(13):1445–1458

Vidt ME. 2014. Muscle structure and function in older adults with a rotator cuff tear. Doctoral Thesis.

Vidt ME, Santago AC, Marsh AP, et al. 2016. The effects of a rotator cuff tear on activities of daily living in older adults: A kinematic analysis. J Biomech, 49(4):611–617.

Wu W, Lee PVS, Bryant AL, et al. 2016. Subject-specific musculoskeletal modeling in the evaluation of shoulder muscle and joint function. J Biomech, 49(15):3626–3634.

Wu G, van der Helm FCT, Veeger HEJ, et al. 2005. ISB recommendation on definitions of joint coordinate systems of various joints for the reporting of human joint motion—Part II: shoulder, elbow, wrist and hand. J Biomech, 38(5):981–992.