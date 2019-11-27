% -------------------------------------------------------------------------- %
% OpenSim Moco: GlenohumeralStability_CustomGoal.m                           %
% -------------------------------------------------------------------------- %
% Copyright (c) 2019 Stanford University and the Authors                     %
%                                                                            %
% Author(s): Aaron Fox (edited from original example by Christopher          %
% Dembia)                                                                    %
%                                                                            %
% Licensed under the Apache License, Version 2.0 (the "License"); you may    %
% not use this file except in compliance with the License. You may obtain a  %
% copy of the License at http://www.apache.org/licenses/LICENSE-2.0          %
%                                                                            %
% Unless required by applicable law or agreed to in writing, software        %
% distributed under the License is distributed on an "AS IS" BASIS,          %
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   %
% See the License for the specific language governing permissions and        %
% limitations under the License.                                             %
% -------------------------------------------------------------------------- %

function value = GlenohumeralStability_CustomGoal
    % This example calculates the glenohumeral stability metric outlined in
    % Chadwick et al. 2014, IEEE Trans Biomed Eng, 61: 1947-1956 to use as a
    % cost/goal function within Moco Simulations. The GH stability metric
    % projects the glenohumeral joint reaction vector onto an ellipse
    % representing the glenoid, and calculates it's relative distance from the
    % centre point of the ellipse. A value of zero represents the JRF vector
    % intersecting the ellipse at the exact centre, while the value approaches
    % 1 as the vector reaches the edge of the ellipse. Litearture based values
    % of humeral head (Knowles et al. 2016, J Shoulder Elbow Surg, 25:
    % 502-509) and glenoid (von Schroeder et al. 2001, Clin Orthop Relat Res,
    % 383: 131-139) anatomy are used in the calculations.
    %
    % The goal of this function is to minimise the glenohumeral stability value
    % at each step of the simulation. Currently the value is calculated
    %
    % In C++, MocoGoals can be in cost mode or endpoint constraint mode. In this
    % example, we expect that your goal is in cost mode.

    import org.opensim.modeling.*;

    %Load the model for the problem
    %The custom goal is written that it will only work with the joints and
    %frames included in this model
    %Add the geometry path first
    ModelVisualizer.addDirToGeometrySearchPaths([pwd,'\Geometry']);
    %Load the model
    osimModel = Model([pwd,'\BaselineModel.osim']);
    osimModel.finalizeConnections();

    %Replace the muscles in the model with DeGrooteFregly2016 muscles
    DeGrooteFregly2016Muscle().replaceMuscles(osimModel);

% % %     %Lock the remaining joints of the model to make this a shoulder only movement
    coordSet = osimModel.updCoordinateSet();
% % %     coordSet.get('thorax_tilt').set_locked(true);
% % %     coordSet.get('thorax_list').set_locked(true);
% % %     coordSet.get('thorax_rotation').set_locked(true);
% % %     coordSet.get('thorax_tx').set_locked(true);
% % %     coordSet.get('thorax_ty').set_locked(true);
% % %     coordSet.get('thorax_tz').set_locked(true);
% % %     coordSet.get('elbow_flexion').set_locked(true);
% % %     coordSet.get('pro_sup').set_locked(true);

    %Finalise model connections
    osimModel.finalizeConnections();

    %Set-up a simple problem to test the cost function. In this instance, it
    %will be for the model to generate a 0-90 degrees abduction motion.

    %Initalise Moco study and problem
    study = MocoStudy();
    problem = study.updProblem();

    %Set the time bounds
    problem.setTimeBounds(0,2);

    %Set the state info for the shoulder coordinates
    problem.setStateInfo(['/jointset/',char(coordSet.get('shoulder_elv').getJoint().getName()),'/',char(coordSet.get('shoulder_elv').getName()),'/value'],...
        MocoBounds(deg2rad(0),deg2rad(90)),MocoInitialBounds(deg2rad(0)),MocoFinalBounds(deg2rad(90)));
    problem.setStateInfo(['/jointset/',char(coordSet.get('shoulder_elv').getJoint().getName()),'/',char(coordSet.get('shoulder_elv').getName()),'/speed'],...
        MocoBounds(deg2rad(-50),deg2rad(50)),MocoInitialBounds(deg2rad(0)),MocoFinalBounds(deg2rad(0)));
    problem.setStateInfo(['/jointset/',char(coordSet.get('elv_angle').getJoint().getName()),'/',char(coordSet.get('elv_angle').getName()),'/value'],...
        MocoBounds(deg2rad(30)),MocoInitialBounds(deg2rad(30)),MocoFinalBounds(deg2rad(30)));
    problem.setStateInfo(['/jointset/',char(coordSet.get('elv_angle').getJoint().getName()),'/',char(coordSet.get('elv_angle').getName()),'/speed'],...
        MocoBounds(deg2rad(0)),MocoInitialBounds(0),MocoFinalBounds(0));
    problem.setStateInfo(['/jointset/',char(coordSet.get('shoulder_rot').getJoint().getName()),'/',char(coordSet.get('shoulder_rot').getName()),'/value'],...
        MocoBounds(deg2rad(0)),MocoInitialBounds(deg2rad(0)),MocoFinalBounds(deg2rad(0)));
    problem.setStateInfo(['/jointset/',char(coordSet.get('shoulder_rot').getJoint().getName()),'/',char(coordSet.get('shoulder_rot').getName()),'/speed'],...
        MocoBounds(deg2rad(0)),MocoInitialBounds(0),MocoFinalBounds(0));

    %Set the state info for muscles
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

    %Add model to problem
    problem.setModel(osimModel);
    
    %Set-up an effort control for the problem
    effort = MocoControlGoal();
    %Append goal to problem
    problem.addGoal(effort);

    %Initialise solver
    solver = study.initCasADiSolver();
    
    %Set number of mesh intervals (lower for a quicker solve)
    solver.set_num_mesh_intervals(10);
    solver.set_optim_convergence_tolerance(1e-3);
    
% % %     %Print the tool to file to check
% % %     study.print('testAbductionProblem.omoco');
    
% % %     %Run solution to get some results for evaluating cost function
% % %     solution = study.solve();
% % %     
% % %     %Set guess from previously "solved" solution
% % %     %Terminated this early so the actual solution ain't great
% % %     solver.setGuessFile([pwd,'\MocoStudy_solution.sto']);
    
% % % %     %Get guess in trajectory format
% % % %     trajectory = solver.getGuess();
    
    %Create guess from solver
    trajectory = solver.createGuess();

    %Evaluate function value
    value = evaluateCustomGoal(problem, trajectory, ...
            @calcMyCustomEffortGoalIntegrand, @calcMyCustomEffortGoalValue);

end

%%

function integrand = calcMyCustomEffortGoalIntegrand(model, state)
    %Compute the integrand for the integral portion of the cost goal.

    %Realise to acceleration stage
    model.realizeAcceleration(state);

    %Calculate joint reaction forces

    %Get the joint to calculate joint reaction force in. This is the unrothum
    %within the model
    unrothum = model.getJointSet().get('unrothum');

    %Calculate the joint reaction force expressed on the parent (i.e. scapula)
    jrf = unrothum.calcReactionOnParentExpressedInGround(state);

    %Separate the force and moment components of JRFs
    %Assuming moment is first Vec3 and forces are second????
    moment = jrf.get(0); force = jrf.get(1);

    %Transform from ground to rotated scapula frame
    ground = model.getGround();
    rotFrame = model.getJointSet().get('unrothum').get_frames(2);
    rotForce = ground.expressVectorInAnotherFrame(state,force,rotFrame);
    
    %Calculate resultant force vector
    resultant = sqrt((rotForce.get(0)^2) + (rotForce.get(1)^2) + (rotForce.get(2)^2));
    
    %Calculate the 3D angles of vector theta and phi
    spherPhi = atand((rotForce.get(0) / rotForce.get(1)));
    spherTheta = acosd((rotForce.get(2) / resultant));
    
    %With these angles and a proposed prescribed distance along the y-axis of
    %this frame, we can work out what the distances from the centre of the
    %glenoid fossa is. Values will be in the same notation as the frame, with
    %positive being up and forward, and negative being down and back.

    %We can set a distance based on a generic radius of the humeral head.
    %Boileau & Walch (1997) found a diameter of 46.2mm for the humeral head;
    %and Knowles et al. (2016) found a diameter of 49mm for the humeral head,
    %slightly bigger and smaller for males and females respectively (but not
    %reported in text, only figure?).

    %Set the distance along the Y-axis to the glenoid fossa based on the data of
    %Knowles et al. (2016) - note it is a radius rather than diameter
    dY = 49/2;
    
    %Calculate distances along the anterior/posterior and vertical axes
    horzPos = tand(spherPhi) * dY;
    vertPos = tand(90-spherTheta) * dY;
    
% % %     %We can now attempt to plot a trace of these results on an ellipse that
% % %     %uses the dimensions of generic scapula anatomy. von Schroeder et al.
% % %     %(2001) reported an AP diameter of 28.6, 25.8 and 30.9 in combined, females
% % %     %and males, respectively; and a SI length of 36.5, 33.6 and 38.0 for
% % %     %combined, females and males, respectively. We'll go with the combined data
% % %     %for now...
% % %     drawEllipse(0,0,28.6/2,36.5/2);
% % %     %Edit the properties of the line object
% % %     h = findobj(gca,'Type','line');
% % %     h.Color = 'k'; h.LineWidth = 2;
% % %     axis equal
% % %     hold on
% % % 
% % %     %Draw the pattern of the original and altered models point of application
% % %     scatter(horzPos,vertPos,'r','filled');
% % % 
% % %     %Close plot
% % %     close all; clear h

    %Calculate GH stability value
    %Find point on the glenoid rim that the vector would intersect in the 2D
    %space - the angles used to reach this point from the GHJ centre will then
    %be used in the calculation of GH stability. This uses a custom function
    %where the inputs are the major axis (i.e. AP diameter/2), minor axis (SI
    %diameter/2), centre coordinates of ellipse, starting point of line (i.e.
    %centre of ellipse in this case), and end point of line (projected point on
    %glenoid).
    [C1,C2] = lineEllipse(28.6/2,36.5/2,[0,0],[0,0],[horzPos,vertPos]);
    %Need to determine which intersection point is required for the
    %calculations, this can be determined by checking the distance between the
    %points and taking whichever is closer.
    C1_dist = dist_markers2D([horzPos,vertPos],C1);
    C2_dist = dist_markers2D([horzPos,vertPos],C2);
    if C2_dist <= C1_dist
        %Use C2 point
        ellipseEdge_horz = C2(1); ellipseEdge_vert = C2(2);
    else
        %Use C1 point
        ellipseEdge_horz = C1(1); ellipseEdge_vert = C1(2);
    end
    clear C1 C2 C1_dist C2_dist
    %Calculate relative distance of projected point from glenoid centre to
    %ellipse edge point and use this as GH stability value
    GHstab = dist_markers2D([horzPos,vertPos],[0,0]) / dist_markers2D([ellipseEdge_horz,ellipseEdge_vert],[0,0]);
    
    %Allocate to integrand function output
    integrand = 0;
    integrand = integrand + GHstab;
    
end

function value = calcMyCustomEffortGoalValue(...
                        model, initial_state, final_state, integral)
    % Compute the goal value from the phase's initial and final states, and from
    % the integral of the integrand function over the phase. The integration is
    % performed for you by evaluateCustomGoal().
    value = integral / (final_state.getTime() - initial_state.getTime());
end

function goalValue = evaluateCustomGoal(...
                        problem, mocoTraj, integrandFunc, goalFunc)
    % Test a custom goal, defined by integrand and goal functions, on the
    % provided problem and MocoTrajectory.

    %Get the model
    model = problem.getPhase(0).getModelProcessor().process();
    
% % %     %Coordinate set seems to need to be unlocked to prescribe the values
% % %     %not exactly matching zero for locked coordinates
% % %     coordSet = model.updCoordinateSet();
% % %     coordSet.get('thorax_tilt').set_locked(false);
% % %     coordSet.get('thorax_list').set_locked(false);
% % %     coordSet.get('thorax_rotation').set_locked(false);
% % %     coordSet.get('thorax_tx').set_locked(false);
% % %     coordSet.get('thorax_ty').set_locked(false);
% % %     coordSet.get('thorax_tz').set_locked(false);
% % %     coordSet.get('elbow_flexion').set_locked(false);
% % %     coordSet.get('pro_sup').set_locked(false);
% % %     model.finalizeConnections();

    %Prescribe controls to model
    org.opensim.modeling.opensimMoco.prescribeControlsToModel(mocoTraj, model);

    %Get the states from trajectory
    statesTraj = mocoTraj.exportToStatesTrajectory(problem);

    %Initialise model
    modelState = model.initSystem();

    %Get size of states
    N = statesTraj.getSize();

    %Loop through state size to compute the integrand at each step
    integrand = zeros(N, 1);
    for i = 0:(N - 1)
        integrand(i + 1) = integrandFunc(model, statesTraj.get(i));
    end

    %%%%%%

    %%%% from example function

    integral = trapz(integrand);
    goalValue = goalFunc(model, statesTraj.front(), statesTraj.back(), integral);

end

