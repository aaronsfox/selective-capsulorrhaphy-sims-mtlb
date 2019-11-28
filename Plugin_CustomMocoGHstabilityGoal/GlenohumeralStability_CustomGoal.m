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
    % 1 as the vector reaches the edge of the ellipse. Literature based values
    % of humeral head (Knowles et al. 2016, J Shoulder Elbow Surg, 25:
    % 502-509) and glenoid (von Schroeder et al. 2001, Clin Orthop Relat Res,
    % 383: 131-139) anatomy are used in the calculations. It would probably
    % be useful to have these as a default, but also give the option to
    % prescribe whatever size 2D ellipse (in mm) the user wants?
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

% % %     %Issue with states coming up as NaN's when coordinates are
% % %     %locked. The locked coordinates don't seem to take on board the state
% % %     %values they are prescribed (i.e. they become NaN's) and this then
% % %     %impacts the calculation of joint reaction forces later in the pipeline
% % %     %(i.e. the output is NaN's). Have left out the locking of coordinates
% % %     %in this example for now - but it would be handy to be able to lock
% % %     %specific coordinates and then still have their states be acknolwedged
% % %     %in joint reaction force calculations.

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
    %Only shoulder states prescribed here due to usually locking other
    %coordinates for such a problem.
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
% % %     %Originally ran this through the solver for a little while (not to
% % %     %completion) to potentially get an appropriate guess that could be used
% % %     %in application of the goal function.
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
            @calcMyCustomGoalIntegrand, @calcMyCustomGoalValue);

end

%%

function integrand = calcMyCustomGoalIntegrand(model, state, ...
    humeralHeadSize, AnteriorPosteriorGlenoidDiameter, SuperiorInferiorGlenoidDiameter)

    %Check for size inputs, if not use defaults
    if nargin < 3
        humeralHeadSize = 49;
    end
    if nargin < 4
        AnteriorPosteriorGlenoidDiameter = 28.6;
    end
    if nargin < 5
        SuperiorInferiorGlenoidDiameter = 36.5;
    end

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
    force = jrf.get(1);

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
	%Currently this is defaulting to he literature-bsaed value, but it may be
	%useful to be able to set this within the goal function (i.e. set the humeral
	%head size variable)
    dY = humeralHeadSize/2;
    
    %Calculate distances along the anterior/posterior and vertical axes
    horzPos = tand(spherPhi) * dY;
    vertPos = tand(90-spherTheta) * dY;

    %Calculate GH stability value
    %Find point on the glenoid rim that the vector would intersect in the 2D
    %space - the angles used to reach this point from the GHJ centre will then
    %be used in the calculation of GH stability. This uses a custom function
    %where the inputs are the major axis (i.e. AP diameter/2), minor axis (SI
    %diameter/2), centre coordinates of ellipse, starting point of line (i.e.
    %centre of ellipse in this case), and end point of line (projected point on
    %glenoid).
	%Like abov with the humeral head size, the glenoid size is coming from
	%literature based values at a default - but may be useful to set these as
	%variables within the goal function
    [C1,C2] = lineEllipse(AnteriorPosteriorGlenoidDiameter/2,SuperiorInferiorGlenoidDiameter/2,[0,0],[0,0],[horzPos,vertPos]);
    %Need to determine which intersection point is required for the
    %calculations, this can be determined by checking the distance between the
    %points and taking whichever is closer.
    C1_dist = distance2D([horzPos,vertPos],C1);
    C2_dist = distance2D([horzPos,vertPos],C2);
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
    GHstab = distance2D([horzPos,vertPos],[0,0]) / distance2D([ellipseEdge_horz,ellipseEdge_vert],[0,0]);
    
    %Allocate to integrand function output
    integrand = 0;
    integrand = integrand + GHstab;
    
end

function value = calcMyCustomGoalValue(...
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

    %Prescribe controls to model
    org.opensim.modeling.opensimMoco.prescribeControlsToModel(mocoTraj, model);

    %Get the states from trajectory
    statesTraj = mocoTraj.exportToStatesTrajectory(problem);

    %Initialise model
    model.initSystem();

    %Get size of states
    N = statesTraj.getSize();

    %Loop through state size to compute the integrand at each step
    integrand = zeros(N, 1);
    for i = 0:(N - 1)
        integrand(i + 1) = integrandFunc(model, statesTraj.get(i));
    end

    %Perform final calculations to extract goal function value
    integral = trapz(integrand);
    goalValue = goalFunc(model, statesTraj.front(), statesTraj.back(), integral);

end

%% Supplementary functions

function [C1,C2] = lineEllipse(a,b,O,A,B)
    % Get points of intersection of line and Ellipse
    % INPUT : a - major axis of Ellipse
    %         b - minor axis of Ellipse
    %         O - center of ellipse i,e (h,k)
    %         A, P  - points of striaght line (x1,y1) and (x2,y2)
    % OUTPUT : C - Point of intersection of line and ellipse
    % Reference : http://www.ambrsoft.com/TrigoCalc/Circles2/Ellipse/EllipseLine.htm
    % 
    % Coded by :    Siva Srinivas Kolukula, PhD      
    %               Tsunami and Storm surge Early WArning Group (TWS)
    %               Indian National Centre for Ocean Information Services (INCOIS)
    %               Hyderabad, INDIA
    % E-mail   :    allwayzitzme@gmail.com                                        
    % web-link :    https://sites.google.com/site/kolukulasivasrinivas/   
    % Ellipse center
    h = O(1) ; k = O(2) ;
    % Line, Get slope and y-intercept of the line
    AB = [A; B];
    line = polyfit(AB(:,1),AB(:,2),1) ;
    m = line(1) ;       % Slope of the line 
    c = line(2) ;       % y-intercept of the line
    % Formula
    eps = c-k ;delta = c+m*h ;
    D = sqrt(a^2*m^2+b^2-delta^2-k^2+2*delta*k) ;
    E = a^2*m^2+b^2 ;
    x1 = (h*b^2-m*a^2*eps+a*b*D)/E ;x2 = (h*b^2-m*a^2*eps-a*b*D)/E ;
    y1 = (b^2*delta+k*a^2*m^2+a*b*m*D)/E ;y2 = (b^2*delta+k*a^2*m^2-a*b*m*D)/E ;
    C1 = [x1 y1] ; C2 = [x2 y2] ;
    % Check if intersection points exists
    if ~isreal(C1)
        C1 = [NaN NaN] ;
        C2 = C1 ;
    end
end

function dist = distance2D(M1,M2)

    % This function calculates the distance between two 2D points
    dist = sqrt((M1(:,1)-M2(:,1)).^2+(M1(:,2)-M2(:,2)).^2);
    
end