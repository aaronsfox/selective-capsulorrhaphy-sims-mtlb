%% Run two simulations, one with and without the constraint to:
%  (1) see if the plugin doesn't crash the simulation
%  (2) see if it makes a difference and does what it's supposed to do

%% Run baseline initial simulation

import org.opensim.modeling.*

%Set main directory
mainDir = pwd;

%Add supplementary code to path
addpath('..\..\Code\Supplementary');

%Go grab the generic shoulder model
cd('..\..\ModelFiles');
modelDir = pwd;
%Add geometry directory
ModelVisualizer.addDirToGeometrySearchPaths([modelDir,'\Geometry']);

%Load the shoulder model
osimModel = Model('FullShoulderModel.osim');
osimModel.finalizeConnections();

%Replace muscles with DeGroote et al. variant
DeGrooteFregly2016Muscle().replaceMuscles(osimModel);

%Lock the thorax joints of the model to make this a shoulder only movement
%Lock the elbow and pronation joints too
%Lock the non elevation shoulder coordinates too
coordSet = osimModel.updCoordinateSet();
coordSet.get('thorax_tilt').set_locked(true);
coordSet.get('thorax_list').set_locked(true);
coordSet.get('thorax_rotation').set_locked(true);
coordSet.get('thorax_tx').set_locked(true);
coordSet.get('thorax_ty').set_locked(true);
coordSet.get('thorax_tz').set_locked(true);
coordSet.get('elbow_flexion').set_locked(true);
coordSet.get('pro_sup').set_locked(true);
coordSet.get('shoulder_rot').set_locked(true);
coordSet.get('elv_angle').setDefaultValue(deg2rad(30));
coordSet.get('elv_angle').set_locked(true);

%Turn off muscle-tendon dynamics to keep the problem simple.
%This is probably already done in the model anyway
for m = 0:osimModel.getMuscles().getSize()-1
    musc = osimModel.updMuscles().get(m);
    musc.set_ignore_tendon_compliance(true);
end
clear m

%Finalise model connections
osimModel.finalizeConnections();

%Set up a Moco study
study = MocoStudy();

%Set up the problem
problem = study.updProblem();

%Set the time bounds on the problem
problem.setTimeBounds(0,[0.5,2]);

%Set the position bounds for the shoulder coordinates
%Shoulder elevation
charValue = ['/jointset/',char(coordSet.get('shoulder_elv').getJoint().getName()),'/',char(coordSet.get('shoulder_elv').getName()),'/value'];
charSpeed = ['/jointset/',char(coordSet.get('shoulder_elv').getJoint().getName()),'/',char(coordSet.get('shoulder_elv').getName()),'/speed'];
problem.setStateInfo(charValue,MocoBounds(deg2rad(0),deg2rad(110)),MocoInitialBounds(deg2rad(0)),MocoFinalBounds(deg2rad(110)));
problem.setStateInfo(charSpeed,MocoBounds(-50,50),MocoInitialBounds(0));
clear charValue charSpeed

% % % %Shoulder rotation
% % % charValue = ['/jointset/',char(coordSet.get('shoulder_rot').getJoint().getName()),'/',char(coordSet.get('shoulder_rot').getName()),'/value'];
% % % charSpeed = ['/jointset/',char(coordSet.get('shoulder_rot').getJoint().getName()),'/',char(coordSet.get('shoulder_rot').getName()),'/speed'];
% % % problem.setStateInfo(charValue,MocoBounds(deg2rad(0)),MocoInitialBounds(deg2rad(0)),MocoFinalBounds(deg2rad(0)));
% % % problem.setStateInfo(charSpeed,MocoBounds(0),MocoInitialBounds(0),MocoFinalBounds(0));
% % % clear mn mx charValue charSpeed
% % % 
% % % %Elevation plane
% % % charValue = ['/jointset/',char(coordSet.get('elv_angle').getJoint().getName()),'/',char(coordSet.get('elv_angle').getName()),'/value'];
% % % charSpeed = ['/jointset/',char(coordSet.get('elv_angle').getJoint().getName()),'/',char(coordSet.get('elv_angle').getName()),'/speed'];
% % % problem.setStateInfo(charValue,MocoBounds(deg2rad(30)),MocoInitialBounds(deg2rad(30)),MocoFinalBounds(deg2rad(30)));
% % % problem.setStateInfo(charSpeed,MocoBounds(0),MocoInitialBounds(0),MocoFinalBounds(0));
% % % clear mn mx charValue charSpeed

% % % % Velocity bounds: all model coordinates should start and end at rest.
% % % problem.setStateInfoPattern('/jointset/.*/speed', [], 0, 0);

%Set the muscle activation state bounds
%Set initial activation to be zero
for m = 0:osimModel.getMuscles().getSize()-1
    %Get current muscle name
    muscName = osimModel.updMuscles().get(m).getName();
    %Create string for setting state info
    stateStr = ['/forceset/',char(muscName),'/activation'];
    %Set the activation bounds
    problem.setStateInfo(stateStr,MocoBounds(0,1),MocoInitialBounds(0));   
    %Cleanup
    clear muscName stateStr
end
clear m

%Set model to the moco problem
problem.setModelCopy(osimModel);

%Add a control cost and final time goal to the problem.
problem.addGoal(MocoControlGoal('effort'));
problem.addGoal(MocoFinalTimeGoal('time'));

%Configure the solver
solver = study.initCasADiSolver();
solver.set_num_mesh_intervals(10);
solver.set_optim_convergence_tolerance(1e-4);
solver.set_optim_constraint_tolerance(1e-4);

%Navigate back to test directory
cd('..\Moco_GHJRF_Constraint\test');

%Print the Moco study to file
study.print('InitialUnconstrainedAbduction.omoco');

%% Solve! Write the solution to file
predictSolutionUnconstrained = study.solve();
% % % predictSolutionUnconstrained.unseal(); 
predictSolutionUnconstrained.write('predictSolutionUnconstrained.sto');
if exist('MocoStudy_solution.sto', 'file') == 2
  delete('MocoStudy_solution.sto');
end

%%%%% for time purposes, stopped the above simulation early, hence the need
%%%%% to unseal

%%%%% TO DO: run to completion



%% Re-run the simulation with the custom path constraint added

%Load the plugin library
cd('..\build\Release');
opensimCommon.LoadOpenSimLibraryExact([pwd,'\osimMocoGHJRFConstraint.dll']);
cd('..\..\test');

%It's tricky to load own plugins programatically, so need to employ a
%sneaky way around this

%Add blank control bound constraint to the problem
pathCon = MocoControlBoundConstraint();
problem.addPathConstraint(pathCon);

%Save and load the .omoco file in XML format
study.print('tempOMOCO.omoco');
[tree,RootName,~] = xml_readOSIM('tempOMOCO.omoco');

%Copy the control bound constraint and then remove it
tree.MocoStudy.MocoProblem.MocoPhase.path_constraints.MocoGHJRFConstraint = ...
    tree.MocoStudy.MocoProblem.MocoPhase.path_constraints.MocoControlBoundConstraint;
tree.MocoStudy.MocoProblem.MocoPhase.path_constraints = ...
    rmfield(tree.MocoStudy.MocoProblem.MocoPhase.path_constraints,'MocoControlBoundConstraint');

%Write it to file
DOMnode = xml_writeOSIM('newOMOCO.omoco',tree,RootName);

%%%% TO DO: solve the issue around the re-saved .omoco file not loading in.
%%%% Solution was to just copy and paste the GHJRF constraint section into
%%%% the old 'temp' one, as this was the only one that would load in as a
%%%% study...

% % % %Load in the new study
% % % newStudy = MocoStudy('tempOMOCO.omoco');

delete tempOMOCO.omoco newOMOCO.omoco

%Print the updated study file
newStudy.print('SecondConstrainedAbduction.omoco');

%Try and solve the new study with the GHJRF constraint included
%And hope that it works...
predictSolutionConstrained = newStudy.solve();

%% For a potentially quicker solution, compare an old predicted solution 
%  to a new one

%%% do this another day




