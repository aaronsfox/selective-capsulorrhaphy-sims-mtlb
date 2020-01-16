function ShoulderCapsulorrhaphySims_3_CreateModels

% @author: Aaron Fox
% Centre for Sport Research, Deakin University
% aaron.f@deakin.edu.au
% 
% This code takes the parameters from the previous scripts optimisation and
% adds the equations into the custom dual expression based force class for 
% the different capsulorrhaphy conditions simulated.
%
% This code also creates visualisations of the force applied by these
% 'ligaments' across the different motions and positions for the different
% capsulorrhaphy conditions simulated. Along with the visualisations, the 
% peak angles achieved across the different capsulorrhaphy conditions are 
% calculated and compared to the Gerber et al. study values.
%
% TO DO: add Gerber et al. references

    %% Set-up
	
	import org.opensim.modeling.*

    %Set main directory
    mainDir = pwd;

    %Add supplementary code folder to path
    addpath('..\Supplementary');
	
	%Load in the Gerber et al. database for joint angles, and angles and
    %plication labels
    %This is created by running the 'createGerber2003_anglesDatabase' function
    %from the supplementary code folder
    cd('..\..\SupportingData');
    load('Gerber2003_AnglesDatabase.mat','angles','meanAngles','plications');
	
	%Navigate to model directory
    cd('..\ModelFiles');
    modelDir = pwd;
    %Add geometry directory
    ModelVisualizer.addDirToGeometrySearchPaths([pwd,'\Geometry']);
	
	%Load the dual expression based force plugin library
	cd('..\Plugin_DualEBCF\build\Release');
    opensimCommon.LoadOpenSimLibraryExact([pwd,'\osimDualEBCF.dll']);	
	
	%Load in the optimisation results
    cd('..\..\..\ModelFiles\OptimisationResults');
	load('OptimisedLigamentParameters.mat','coordForceFunc','ligParamOpt','optResults');
    
    %% Create basic versions of capsulorrhaphy models
    
    %Navigate to model directory
    cd('..');
    
    %Classes developed from plugins are difficult to work in Matlab with
    %using the API, so the workaround to this is to work with the model
    %files as text or XML-bsaed files.
    
    %Read in the basic shoulder complex with the blank dual expression
    %based coordinate force to fill with data from the optimisation.
    [baseModelXML, RootName, ~] = xml_readOSIM('BasicShoulderComplex_withBlankDualEBCF.osim');
    
    %Set preferences for writing to avoid itemisation of forces
    Pref.StructItem = false;
    
    %Create damping forces for the model coordinates to add in each time
    
    %Shoulder elevation damping CLF
    elvDamp = CoordinateLimitForce();            %constructor with default properties
    elvDamp.setName('shoulder_elv_damping');     %set name
    elvDamp.set_coordinate('shoulder_elv');      %set coordinate
    elvDamp.set_upper_stiffness(1e-08);          %set upper stiffness value
    elvDamp.set_upper_limit(190);                %set upper limit threshold
    elvDamp.set_lower_stiffness(1e-08);          %set lower stiffness value
    elvDamp.set_lower_limit(190);                %set lower limit threshold
    elvDamp.set_damping(0.001745);               %set damping value
    elvDamp.set_transition(1);                   %set transition value
        
    %Shoulder rotation damping CLF
    rotDamp = CoordinateLimitForce();            %constructor with default properties
    rotDamp.setName('shoulder_rot_damping');     %set name
    rotDamp.set_coordinate('shoulder_rot');      %set coordinate
    rotDamp.set_upper_stiffness(1e-08);          %set upper stiffness value
    rotDamp.set_upper_limit(190);                %set upper limit threshold
    rotDamp.set_lower_stiffness(1e-08);          %set lower stiffness value
    rotDamp.set_lower_limit(190);                %set lower limit threshold
    rotDamp.set_damping(0.001745);               %set damping value
    rotDamp.set_transition(1);                   %set transition value
    
    %Set the parameters for the two coordinate forces in each model
    for pp = 1:length(plications)

        %Create a copy of the base model to edit
        copyModelXML = baseModelXML;

        %Shoulder elevation (only need positive expression for this one)
        copyModelXML.Model.ForceSet.objects.DualExpressionBasedCoordinateForce(1).coordinate = 'shoulder_elv';
        copyModelXML.Model.ForceSet.objects.DualExpressionBasedCoordinateForce(1).coordinateDual = 'elv_angle';
        copyModelXML.Model.ForceSet.objects.DualExpressionBasedCoordinateForce(1).expressionPos = coordForceFunc.(plications{pp}).Elv;

        %Shoulder rotation
        copyModelXML.Model.ForceSet.objects.DualExpressionBasedCoordinateForce(2).coordinate = 'shoulder_rot';
        copyModelXML.Model.ForceSet.objects.DualExpressionBasedCoordinateForce(2).coordinateDual = 'shoulder_elv';
        copyModelXML.Model.ForceSet.objects.DualExpressionBasedCoordinateForce(2).expressionPos = coordForceFunc.(plications{pp}).IntRot;
        copyModelXML.Model.ForceSet.objects.DualExpressionBasedCoordinateForce(2).expressionNeg = coordForceFunc.(plications{pp}).ExtRot;

        %Rename model
        copyModelXML.Model.ATTRIBUTE.name = ['BasicCapsModel_',plications{pp}];
        
        %Rename the expression force objects
        copyModelXML.Model.ForceSet.objects.DualExpressionBasedCoordinateForce(1).ATTRIBUTE.name = 'shoulder_elv_DualEBCF';
        copyModelXML.Model.ForceSet.objects.DualExpressionBasedCoordinateForce(2).ATTRIBUTE.name = 'shoulder_rot_DualEBCF';

        %Export edited .osim file from XML
        xml_writeOSIM(['BasicCapsModel_',plications{pp},'.osim'],copyModelXML,RootName,Pref);
        
        %Bring the model back in to add the damping force for ligaments
        editModel.(plications{pp}) = Model(['BasicCapsModel_',plications{pp},'.osim']);
        
        %Add the damping forces to model
        editModel.(plications{pp}).addForce(elvDamp)
        editModel.(plications{pp}).addForce(rotDamp)
        
        %Finalise connections
        editModel.(plications{pp}).finalizeConnections();
        
        %Re-save the model
        editModel.(plications{pp}).print(['BasicCapsModel_',plications{pp},'.osim']);
        
        %Cleanup
        clear copyModelXML
        
    end
    clear pp
    
    %% Run simulations with each model similar to the optimisation procedure
    %  to ensure that the desired end joint positions are being achieved by
    %  each capsulorrhaphy model
    
    %Create variables that set the starting/locked joint angles for the
    %different simulation positions. This variable is order in columns of:
    %shoulder elevation, elevation angle and shoulder rotation
    startingAngles = [0, 30, 0;
        0, 90, 0;
        0, 30, 0;
        0, 30, 0;
        45, 30, 0;
        45, 30, 0;
        90, 30, 0;
        90, 30, 0];
    
    coordLock = [false, true, true;
        false, true, true;
        true, true, false;
        true, true, false;
        true, true, false;
        true, true, false;
        true, true, false;
        true, true, false];
    
    %Set an empty variable to store the end joint angles and errors in
    expAngles = zeros(length(plications),length(angles));
    
    %Create a directory to store optimised results in
    mkdir('OptimisedResults');
    
    %Loop through the different angles and models to run forward simulations
    for pp = 1:length(plications)
        for aa = 1:length(angles)
            
            %Load in the current model
            osimModel = Model(['BasicCapsModel_',plications{pp},'.osim']);
            
            %Set the motion being examined
            motion = angles{aa};
            
            %Set the default coordinate values for the starting positions
            %based on the motion being tested
            osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(startingAngles(aa,1)));
            osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(startingAngles(aa,2)));
            osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(startingAngles(aa,3)));
            
            %Lock the relevant coordintes based on the motion being tested
            osimModel.getCoordinateSet().get('shoulder_elv').set_locked(coordLock(aa,1));
            osimModel.getCoordinateSet().get('elv_angle').set_locked(coordLock(aa,2));
            osimModel.getCoordinateSet().get('shoulder_rot').set_locked(coordLock(aa,3));
            
            %Add a force reporter analysis to the model for the forward sim
            FrAnalysis = ForceReporter();
            if contains(motion,'Rot')
                FrAnalysis.setStartTime(0); FrAnalysis.setEndTime(3);
            else
                FrAnalysis.setStartTime(0); FrAnalysis.setEndTime(5);
            end
            osimModel.getAnalysisSet().cloneAndAppend(FrAnalysis);
            
            %Finalise model connections
            osimModel.finalizeConnections();
            
            %Initialise the forward tool
            FwdTool = ForwardTool();

            %Settings for forward tool
            FwdTool.setModel(osimModel);

            %Set times based on motion. Rotations = 3 seconds; Elevations = 5 seconds
            if contains(motion,'Rot')
                FwdTool.setInitialTime(0); FwdTool.setFinalTime(3);
            else
                FwdTool.setInitialTime(0); FwdTool.setFinalTime(5);
            end
            
            %Set controls for simulation, depending on whether it is an elevation or
            %internal/external rotation motion
            if contains(motion,'IntRot')
                FwdTool.setControlsFileName([modelDir,'\shoulder_introt_controls.xml']);
            elseif contains(motion,'ExtRot')
                FwdTool.setControlsFileName([modelDir,'\shoulder_extrot_controls.xml']);
            else
                FwdTool.setControlsFileName([modelDir,'\shoulder_elv_controls.xml']);
            end
            
            %Need to add controller set for the controls to be active in the model
            %First, clear any existing controller set from past simulations
            osimModel.getControllerSet().clearAndDestroy();
            %Add the current controller set to the model
            FwdTool.addControllerSetToModel();
            
            %Rename the forward tool based on parameters
            FwdTool.setName(motion);
            
            %Navigate to results folder
            cd('OptimisedResults');
            
            %If on first run through for model, create directory for its results
            if aa == 1
                mkdir(plications{pp})
            end
            
            %Navigate to plication directory
            cd(plications{pp});

            %Run forward tool
            FwdTool.run();

            %Load in states and force data
            statesD = importdata([motion,'_states_degrees.mot']);
            forceD = importdata([motion,'_ForceReporter_forces.sto']);

            %Grab out the relevant shoulder angle and force data
            if contains(motion,'Rot')
                angInd = contains(statesD.colheaders,'shoulder_rot') & contains(statesD.colheaders,'value');
                forInd = strcmp(forceD.colheaders,'shoulder_rot_DualEBCF');
            else
                angInd = contains(statesD.colheaders,'shoulder_elv') & contains(statesD.colheaders,'value');
                forInd = strcmp(forceD.colheaders,'shoulder_elv_DualEBCF');
            end
            
            %Get and store the shoulder angle and force data
            simResults.(plications{pp}).(angles{aa}).shoulderAngle = statesD.data(:,angInd);
            simResults.(plications{pp}).(angles{aa}).ligForce = forceD.data(:,forInd);
            
            %Calculate and store peak angles and errors
            if contains(motion,'IntRot')
                expAngles(pp,aa) = max(simResults.(plications{pp}).(angles{aa}).shoulderAngle);                
            elseif contains(motion,'ExtRot')
                expAngles(pp,aa) = min(simResults.(plications{pp}).(angles{aa}).shoulderAngle);
            else
                expAngles(pp,aa) = max(simResults.(plications{pp}).(angles{aa}).shoulderAngle);                
            end
            
            %Cleanup
            clear angInd forInd statesD forceD osimModel motion FrAnalysis FwdTool
            
            %Navigate back to base directory
            cd('..\..');

        end
        clear aa
    end
    clear pp
    
    %Calculate difference between Gerber et al. angles and experimental angles
    errAngles = expAngles - meanAngles;
    pErrAngles = errAngles ./ meanAngles * 100;
    
    %% Create plots of 'ligament' forces vs. shoulder angles
    
    %These plots will be visualised across a relevant spectrum of joint
    %angles, rather than what is achieved during the simulations - and
    %therefore will be calculated from the coordinate functions
    
    %First convert strings to function handles
    for pp = 1:length(plications)
        coordForceFunc.(plications{pp}).f_IntRot = ...
            str2func(append('@(q1,q2)',convertCharsToStrings(coordForceFunc.(plications{pp}).IntRot)));
        coordForceFunc.(plications{pp}).f_ExtRot = ...
            str2func(append('@(q1,q2)',convertCharsToStrings(coordForceFunc.(plications{pp}).ExtRot)));
        coordForceFunc.(plications{pp}).f_Elv = ...
            str2func(append('@(q1,q2)',convertCharsToStrings(coordForceFunc.(plications{pp}).Elv)));
    end
    clear pp
    
    
    
    %Abduction (i.e. elevation at 30 degrees elevation plane angle)
    
    %Initialise figure
    figure
    hold on
    
    %Loop through plications
    for pp = 1:length(plications)
        
        %Create variables for q1 and q2 inputs of function
        %Take the maximum angle that is achieved for abduction (aa=1)
        %%%%% TO DO: fix this with an angles loop...
        shoulderElv = deg2rad(linspace(0,expAngles(pp,1),100));
        elvAngle = deg2rad(ones(1,100)*30);
        
        %Evaluate function at each step
        y = zeros(length(shoulderElv),1);
        for tt = 1:length(shoulderElv)
            y(tt) = feval(coordForceFunc.(plications{pp}).f_Elv,shoulderElv(tt),elvAngle(tt));
        end
        
        %Plot force against shoulder angle 
        %Invert as elevation force produces negative value
        plot(rad2deg(shoulderElv),y*-1,'LineWidth',1.5);
        
        %Cleanup
        clear y shoulderElv elvAngle
        
    end
    clear pp
    
    %Set x-axis limits to peak angle reached
    xlim([0,max(expAngles(:,1))]); %%%%%column 1 for first angle
    
    %Set legend   
    legend(plications,'Location','NorthWest');
    
    
    %%%% TO DO: fix colours...
    
    

    
	
%% Goal of this function is to create the models with the optimised 
%  parameters and visualise some curves that represent the ligament forces
%
%  Consider plotting the total ligament function force versus the relevant
%  joint angle. Consider plotting joint angle on the x-axis and only
%  plotting the function value to the joint angle that is achieved via the
%  separate plications. Theoretically we should see shorter but steeper
%  curves for plications...