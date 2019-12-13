function ShoulderCapsulorrhaphySims_2_OptimiseLigamentParameters

% @author: Aaron Fox
% Centre for Sport Research, Deakin University
% aaron.f@deakin.edu.au
% 
% This code serves to optimise the parameters of the coordinate limit force
% 'ligaments' so that the joint angles reached by the shoulder during
% passively generate movement align with those presented in Gereber et al.
% (2003). The joint angle vs. force outputs from the optimised CLF
% parameters are subsequently used to fit representative equations that are
% used in the dual expression based coordinate force plugin class - to
% provide the final 'ligament' forces to the model.
% 
% TO DO: Notes on simulation time (see innerLevel_BushingOptimisation.m doc
% for notes on this plus other info...)
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
    %Set the Gerber et al. data as global variables
    global meanAngles angles plications
    load('Gerber2003_AnglesDatabase.mat','angles','meanAngles','plications');

    %Navigate to model directory
    cd('..\ModelFiles');
    modelDir = pwd;
    %Add geometry directory
    ModelVisualizer.addDirToGeometrySearchPaths([pwd,'\Geometry']);

    %Get model file
    ModelFile = [pwd,'\BasicShoulderComplex_withForces.osim'];

    %Set-up the model for forward simulations within optimisation
    global osimModel
    osimModel = Model(ModelFile);

    %% Run the initial optimisations for the 'None' model - as this will serve
    %  as the baseline parameters for subsequent optimisations
    plication = 'None';

    %Navigate to optimisation results directory
    cd('OptimisationResults');

    %% Loop through each of the angles/motions to optimise the CLF parameters
    for mm = 1:length(angles)

        %Set the motion variable for optimisations
        motion = angles{mm};

        %Setup inputs for optimisation function
        Input.plicationType = plication;
        Input.motionType = motion;
        Input.modelDir = modelDir;

        %Set the starting parameters and bounds for optimisation, dependent on
        %what motion is being examined. Note that on iterations subsequent to
        %the first for elevation or rotation motions, the parameters will be
        %taken from the previous optimisation of the movement - in theory, this
        %should speed up the optimisation as the parameters initial guess will
        %likely be closer to what its optimised value should be.

        %Check if its an internal rotation motion
        if contains(motion,'IntRot')

            %Set initial guess based on model parameters

            %The variables input to the optimisation are:
            %[1] Shoulder rotation ligament upper limit
            %[2] Shoulder rotation ligament upper stiffness
            %[3] Shoulder rotation ligament transition
            x0 = zeros(3,1);
            x0(1) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).get_upper_limit();
            x0(2) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).get_upper_stiffness();
            x0(3) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).get_transition();

            %Set the bounds
            xUB = zeros(3,1); xLB = zeros(3,1);
            %Upper bounds
            xUB(1) = 110; xUB(2) = 200; xUB(3) = 600;
            %Lower bounds
            xLB(1) = 1; xLB(2) = 5; xLB(3) = 400;

        %Check if its an external rotation motion
        elseif contains(motion,'ExtRot')

            %Set initial guess based on model parameters

            %The variables input to the optimisation are:
            %[1] Shoulder rotation ligament lower limit
            %[2] Shoulder rotation ligament lower stiffness
            %[3] Shoulder rotation ligament transition
            x0 = zeros(3,1);
            x0(1) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).get_lower_limit();
            x0(2) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).get_lower_stiffness();
            x0(3) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).get_transition();

            %Set the bounds
            xUB = zeros(3,1); xLB = zeros(3,1);
            %Upper bounds
            xUB(1) = -1; xUB(2) = 200; xUB(3) = 600;
            %Lower bounds
            xLB(1) = -140; xLB(2) = 5; xLB(3) = 400;

        %Its an elevation motion
        else

            %Set initial guess based on model parameters

            %The variables input to the optimisation are:
            %[1] Shoulder elevation ligament upper limit
            %[2] Shoulder elevation ligament upper stiffness
            %[3] Shoulder elevation ligament transition
            x0 = zeros(3,1);
            x0(1) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_elv_ligaments')).get_upper_limit();
            x0(2) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_elv_ligaments')).get_upper_stiffness();
            x0(3) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_elv_ligaments')).get_transition();

            %Set the bounds
            xUB = zeros(3,1); xLB = zeros(3,1);
            %Upper bounds
            xUB(1) = 180; xUB(2) = 200; xUB(3) = 600;
            %Lower bounds
            xLB(1) = 1; xLB(2) = 5; xLB(3) = 400;

        end

        %% Run the optimisation

        %%%% TO DO: figure out how to supress constant outputs from opensim tools
        %%%% TO DO: potentially write optimisation steps to file

        %Set optimisation options
        options = optimset('fminsearch');
        options.PlotFcns = @optimplotfval;

        %Create folder for current motion optimisation
        mkdir(motion); cd(motion);

        %Start a timer for optimisation
        tStart = tic;

        %Run the optimisation
        [ligParamOpt.(char(plication)).(char(motion)).xOpt,...
            ligParamOpt.(char(plication)).(char(motion)).fval,...
            ligParamOpt.(char(plication)).(char(motion)).exitflag,...
            ligParamOpt.(char(plication)).(char(motion)).output] = ...
            fminsearchbnd('ligamentOptimiser',x0,xLB,xUB,options,Input);

        tEnd = toc(tStart);

        %Save fminsearch figure
        h = gcf;
        title({[motion,' optimisation for "',plication,'" model.'],...
            ['End function value: ',num2str(ligParamOpt.(char(plication)).(char(motion)).fval)]});
        set(h,'PaperPositionMode','auto')
        saveas(h,[motion,'_Optimisation_',plication,'Model.fig']);
        saveas(h,[motion,'_Optimisation_',plication,'Model.png']);
        close all

        %Export optimisation results as text file
        fid = fopen([motion,'_Optimisation_',plication,'Model_Results.txt'],'wt');
        fprintf(fid,['%d minutes and %f seconds on computer: ',getenv('COMPUTERNAME'),'\n'], floor(tEnd/60), rem(tEnd,60));
        if contains(motion,'IntRot')
            fprintf(fid,['Shoulder rotation ligament upper limit: ',num2str(ligParamOpt.(char(plication)).(char(motion)).xOpt(1)),'\n']);
            fprintf(fid,['Shoulder rotation upper stiffness: ',num2str(ligParamOpt.(char(plication)).(char(motion)).xOpt(2)),'\n']);
            fprintf(fid,['Shoulder rotation transition: ',num2str(ligParamOpt.(char(plication)).(char(motion)).xOpt(3))]);
        elseif contains(motion,'ExtRot')
            fprintf(fid,['Shoulder rotation ligament lower limit: ',num2str(ligParamOpt.(char(plication)).(char(motion)).xOpt(1)),'\n']);
            fprintf(fid,['Shoulder rotation lower stiffness: ',num2str(ligParamOpt.(char(plication)).(char(motion)).xOpt(2)),'\n']);
            fprintf(fid,['Shoulder rotation transition: ',num2str(ligParamOpt.(char(plication)).(char(motion)).xOpt(3))]);
        else
            fprintf(fid,['Shoulder elevation ligament upper limit: ',num2str(ligParamOpt.(char(plication)).(char(motion)).xOpt(1)),'\n']);
            fprintf(fid,['Shoulder elevation upper stiffness: ',num2str(ligParamOpt.(char(plication)).(char(motion)).xOpt(2)),'\n']);
            fprintf(fid,['Shoulder elevation transition: ',num2str(ligParamOpt.(char(plication)).(char(motion)).xOpt(3))]);
        end
        fclose(fid);

        %Cleanup
        clear h ax fid tStart tEnd

        %Exit out of directory
        cd('..');

        %Cleanup
        clear ans motion x0 xUB xLB

    end
    clear mm

    %% Use the optimisation parameters from corresponding motions to establish
    %  combined expressions that use the two coordinates relating to the
    %  specific motion (e.g. elevation and elevation angle; rotation and
    %  shoulder elevation) to develop a function that can be used in the dual
    %  expression based coordinate force.

    %% Start with the internal rotation movements

    %Run simulations with each of the optimised rotation ligament parameters to
    %get the force-angle profiles out

    %Set a variable for rotation motion outputs
    rotMots = [{'IntRot0'}; {'IntRot45'}; {'IntRot90'}];

    %Loop through the motions
    for rr = 1:length(rotMots)

        %Navigate to optimisation directory for motion
        cd(rotMots{rr});

        %Set the optimised input parameters
        x = ligParamOpt.(char(plication)).(rotMots{rr}).xOpt;

        %Set parameters
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_upper_limit(x(1));
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_upper_stiffness(x(2));
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_transition(x(3));

        %Run the forward simulation
        %Set the shoulder coordinate values
        if contains(rotMots{rr},'Rot0')
            %Set the shoulder coordinate values
            osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(30));
            osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(0));
            osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(0));
        elseif contains(rotMots{rr},'Rot45')
            %Set the shoulder coordinate values
            osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(30));
            osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(45));
            osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(0));        
        elseif contains(rotMots{rr},'Rot90')
            %Set the shoulder coordinate values
            osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(30));
            osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(90));
            osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(0)); 
        end

        %Set locking on shoulder coordinates
        osimModel.getCoordinateSet().get('elv_angle').set_locked(true);
        osimModel.getCoordinateSet().get('shoulder_elv').set_locked(true);
        osimModel.getCoordinateSet().get('shoulder_rot').set_locked(false);

        %Finalise model connections
        osimModel.finalizeConnections();

        %Initialise the forward tool
        FwdTool = ForwardTool();

        %Settings for forward tool
        FwdTool.setModel(osimModel);
        FwdTool.setInitialTime(0); FwdTool.setFinalTime(3);

        %Set controls for simulation
        FwdTool.setControlsFileName([modelDir,'\shoulder_introt_controls.xml']);

        %Need to add controller set for the controls to be active in the model
        %First, clear any existing controller set from past simulations
        osimModel.getControllerSet().clearAndDestroy();
        %Add the current controller set to the model
        FwdTool.addControllerSetToModel();

        %Run forward tool
        FwdTool.run();

        %Run a force reporter analysis tool on the forward simulation results
        %Printing of model and tool seems necessary as running from Matlab command
        %doesn't recognise states file added in without doing this
        AnTool = AnalyzeTool(); AnTool.setModelFilename('tempModel.osim');
        AnTool.setStatesFileName('_states.sto');
        AnTool.setStartTime(0); AnTool.setFinalTime(3);
        FrAnalysis = ForceReporter();
        FrAnalysis.setStartTime(0); FrAnalysis.setEndTime(3);
        AnTool.getAnalysisSet().cloneAndAppend(FrAnalysis);
        osimModel.print('tempModel.osim'); AnTool.print('Analyse.xml');
        system('opensim-cmd run-tool Analyse.xml');
        %Cleanup
        delete Analyse.xml tempModel.osim
        clear AnTool FrAnalysis FwdTool

        %Load in states and force data
        statesD = importdata('_states.sto');
        forceD = importdata('_ForceReporter_forces.sto');

        %Grab out the shoulder rotation angle
        angInd = contains(statesD.colheaders,'shoulder_rot') & contains(statesD.colheaders,'value');
        optResults.(char(plication)).(rotMots{rr}).rotAngle = statesD.data(:,angInd);
        clear angInd statesD

        %Grab out the ligament force data
        forInd = contains(forceD.colheaders,'shoulder_rot_ligaments');
        optResults.(char(plication)).(rotMots{rr}).ligForce = forceD.data(:,forInd);
        clear forInd forceD

        %Cleanup files from simulationds
        delete _controls.sto _ForceReporter_forces.sto _states.sto _states_degrees.mot

        %Fita one term Gaussian model to the angle-force data
        %Found this model to perform best in matching the data points from the
        %forward simulations. Exponential curves didn't quite fit the curviness
        %as well as the Gaussian option
        optResults.(char(plication)).(rotMots{rr}).gaussFunc = ...
            fit(optResults.(char(plication)).(rotMots{rr}).rotAngle,optResults.(char(plication)).(rotMots{rr}).ligForce,'gauss1');
        %Plot and save exponential fit
        plot(optResults.(char(plication)).(rotMots{rr}).gaussFunc,optResults.(char(plication)).(rotMots{rr}).rotAngle,optResults.(char(plication)).(rotMots{rr}).ligForce);
        funcText = sprintf('f(x) = %s*exp(-((x-%s)/%s)^2)',...
            num2str(optResults.(char(plication)).(rotMots{rr}).gaussFunc.a1), num2str(optResults.(char(plication)).(rotMots{rr}).gaussFunc.b1),...
            num2str(optResults.(char(plication)).(rotMots{rr}).gaussFunc.c1));
        title({['Gaussian fit for ',(rotMots{rr}),' optimisation for "',plication,'" model.'],funcText});
        legend('Location','SouthWest');
        xlabel('Internal Shoulder Rotation (radians)'); ylabel('Ligament Force');
        h = gcf; set(h,'PaperPositionMode','auto');
        saveas(h,[rotMots{rr},'_GaussianFit_',plication,'Model.fig']);
        saveas(h,[rotMots{rr},'_GaussianFit_',plication,'Model.png']);
        close all; clear h funcText

        %Extract the coefficients from the curve fit
        optResults.(char(plication)).(rotMots{rr}).coeff.a = optResults.(char(plication)).(rotMots{rr}).gaussFunc.a1;
        optResults.(char(plication)).(rotMots{rr}).coeff.b = optResults.(char(plication)).(rotMots{rr}).gaussFunc.b1;
        optResults.(char(plication)).(rotMots{rr}).coeff.c = optResults.(char(plication)).(rotMots{rr}).gaussFunc.c1;

        %Cleanup
        clear x ans

        %Navigate back to optimisation directory
        cd('..');

    end
    clear rr rotMots

    %The next step is to model the 3 coefficients from the Gaussian models (a,
    %b and c) against shoulder elevation. This will therefore allow the
    %coefficients for the equations that produce the ligament force to be
    %modelled as a function of the two coordinates.

    %Create a new directory to store results
    mkdir('IntRot_Elv'); cd('IntRot_Elv');

    %Provide a variable for shoulder elevation angle
    sElv = [deg2rad(0);deg2rad(45);deg2rad(90)];

    %Extract coefficients into variables
    coeffA = [optResults.(char(plication)).IntRot0.coeff.a
        optResults.(char(plication)).IntRot45.coeff.a;
        optResults.(char(plication)).IntRot90.coeff.a];
    coeffB = [optResults.(char(plication)).IntRot0.coeff.b
        optResults.(char(plication)).IntRot45.coeff.b;
        optResults.(char(plication)).IntRot90.coeff.b];
    coeffC = [optResults.(char(plication)).IntRot0.coeff.c
        optResults.(char(plication)).IntRot45.coeff.c;
        optResults.(char(plication)).IntRot90.coeff.c];

    %Based on an initial look at IR and None model results, the best fits are:
        % a, b & c = 2nd order (i.e. quadratic) polynomial
    %If this doesn't hold across different plication conditions then it would
    %be a matter of fitting a bunch of different equations and identifying the
    %one that had the least error/highest R-squared.

    %Fit quadratic polynomial curves to coefficients

    %Coeff a
    optResults.(char(plication)).IntRot_Elv.aFunc = fit(sElv,coeffA,'poly2');
    %Plot and save results
    plot(optResults.(char(plication)).IntRot_Elv.aFunc,sElv,coeffA);
    funcText = sprintf('f(x) = %s*x^2 + %s*x + %s',...
        num2str(optResults.(char(plication)).IntRot_Elv.aFunc.p1), num2str(optResults.(char(plication)).IntRot_Elv.aFunc.p2), num2str(optResults.(char(plication)).IntRot_Elv.aFunc.p3));
    title({['Quadratic fit for IntRot coefficient ''a'' for "',plication,'" model.'],funcText});
    legend('Location','NorthEast');
    xlabel('Shoulder Elevation (radians)'); ylabel('Function Value');
    h = gcf; set(h,'PaperPositionMode','auto');
    saveas(h,['IntRot_coeffA_QuadraticFit_',plication,'Model.fig']);
    saveas(h,['IntRot_coeffA_QuadraticFit_',plication,'Model.png']);
    close all; clear h funcText

    %Coeff b
    optResults.(char(plication)).IntRot_Elv.bFunc = fit(sElv,coeffB,'poly2');
    %Plot and save results
    plot(optResults.(char(plication)).IntRot_Elv.bFunc,sElv,coeffB);
    funcText = sprintf('f(x) = %s*x^2 + %s*x + %s',...
        num2str(optResults.(char(plication)).IntRot_Elv.bFunc.p1), num2str(optResults.(char(plication)).IntRot_Elv.bFunc.p2), num2str(optResults.(char(plication)).IntRot_Elv.bFunc.p3));
    title({['Quadratic fit for IntRot coefficient ''b'' for "',plication,'" model.'],funcText});
    legend('Location','NorthEast');
    xlabel('Shoulder Elevation (radians)'); ylabel('Function Value');
    h = gcf; set(h,'PaperPositionMode','auto');
    saveas(h,['IntRot_coeffB_QuadraticFit_',plication,'Model.fig']);
    saveas(h,['IntRot_coeffB_QuadraticFit_',plication,'Model.png']);
    close all; clear h funcText

    %Coeff c
    optResults.(char(plication)).IntRot_Elv.cFunc = fit(sElv,coeffC,'poly2');
    %Plot and save results
    plot(optResults.(char(plication)).IntRot_Elv.cFunc,sElv,coeffC);
    funcText = sprintf('f(x) = %s*x^2 + %s*x + %s',...
        num2str(optResults.(char(plication)).IntRot_Elv.cFunc.p1), num2str(optResults.(char(plication)).IntRot_Elv.cFunc.p2), num2str(optResults.(char(plication)).IntRot_Elv.cFunc.p3));
    title({['Quadratic fit for IntRot coefficient ''c'' for "',plication,'" model.'],funcText});
    legend('Location','NorthWest');
    xlabel('Shoulder Elevation (radians)'); ylabel('Function Value');
    h = gcf; set(h,'PaperPositionMode','auto');
    saveas(h,['IntRot_coeffC_QuadraticFit_',plication,'Model.fig']);
    saveas(h,['IntRot_coeffC_QuadraticFit_',plication,'Model.png']);
    close all; clear h funcText

    %We can now take the general one component Gaussian formula fit to each
    %of the internal rotation motions: a*exp(-((x-b)/c)^2); 
    %where x = internal rotation angle (i.e. q1), and substitute in the
    %functions for each of the coefficients, where x = shoulder elevation (i.e.
    %q2). This needs to have no white space for the opensim formatting.
    aTxt = sprintf('(%s*q2^2+%s*q2+%s)',...
        num2str(optResults.(char(plication)).IntRot_Elv.aFunc.p1), num2str(optResults.(char(plication)).IntRot_Elv.aFunc.p2), num2str(optResults.(char(plication)).IntRot_Elv.aFunc.p3));
    bTxt = sprintf('(%s*q2^2+%s*q2+%s)',...
        num2str(optResults.(char(plication)).IntRot_Elv.bFunc.p1), num2str(optResults.(char(plication)).IntRot_Elv.bFunc.p2), num2str(optResults.(char(plication)).IntRot_Elv.bFunc.p3));
    cTxt = sprintf('(%s*q2^2+%s*q2+%s)',...
        num2str(optResults.(char(plication)).IntRot_Elv.cFunc.p1), num2str(optResults.(char(plication)).IntRot_Elv.cFunc.p2), num2str(optResults.(char(plication)).IntRot_Elv.cFunc.p3));
    coordForceFunc.(char(plication)).IntRot = [aTxt,'*exp(-((q1-',bTxt,')/',cTxt,')^2)'];

    %Navigate back to optimisation directory
    cd('..');

    %Cleanup
    clear aTxt bTxt cTxt coeffA coeffB coeffC sElv

    %% Next do external rotation movements

    %Run simulations with each of the optimised rotation ligament parameters to
    %get the force-angle profiles out

    %Set a variable for rotation motion outputs
    rotMots = [{'ExtRot0'}; {'ExtRot45'}; {'ExtRot90'}];

    %Loop through the motions
    for rr = 1:length(rotMots)

        %Navigate to optimisation directory for motion
        cd(rotMots{rr});

        %Set the optimised input parameters
        x = ligParamOpt.(char(plication)).(rotMots{rr}).xOpt;

        %Set parameters
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_lower_limit(x(1));
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_lower_stiffness(x(2));
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_transition(x(3));

        %Run the forward simulation
        %Set the shoulder coordinate values
        if contains(rotMots{rr},'Rot0')
            %Set the shoulder coordinate values
            osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(30));
            osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(0));
            osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(0));
        elseif contains(rotMots{rr},'Rot45')
            %Set the shoulder coordinate values
            osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(30));
            osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(45));
            osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(0));        
        elseif contains(rotMots{rr},'Rot90')
            %Set the shoulder coordinate values
            osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(30));
            osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(90));
            osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(0)); 
        end

        %Set locking on shoulder coordinates
        osimModel.getCoordinateSet().get('elv_angle').set_locked(true);
        osimModel.getCoordinateSet().get('shoulder_elv').set_locked(true);
        osimModel.getCoordinateSet().get('shoulder_rot').set_locked(false);

        %Finalise model connections
        osimModel.finalizeConnections();

        %Initialise the forward tool
        FwdTool = ForwardTool();

        %Settings for forward tool
        FwdTool.setModel(osimModel);
        FwdTool.setInitialTime(0); FwdTool.setFinalTime(3);

        %Set controls for simulation
        FwdTool.setControlsFileName([modelDir,'\shoulder_extrot_controls.xml']);

        %Need to add controller set for the controls to be active in the model
        %First, clear any existing controller set from past simulations
        osimModel.getControllerSet().clearAndDestroy();
        %Add the current controller set to the model
        FwdTool.addControllerSetToModel();

        %Run forward tool
        FwdTool.run();

        %Run a force reporter analysis tool on the forward simulation results
        %Printing of model and tool seems necessary as running from Matlab command
        %doesn't recognise states file added in without doing this
        AnTool = AnalyzeTool(); AnTool.setModelFilename('tempModel.osim');
        AnTool.setStatesFileName('_states.sto');
        AnTool.setStartTime(0); AnTool.setFinalTime(3);
        FrAnalysis = ForceReporter();
        FrAnalysis.setStartTime(0); FrAnalysis.setEndTime(3);
        AnTool.getAnalysisSet().cloneAndAppend(FrAnalysis);
        osimModel.print('tempModel.osim'); AnTool.print('Analyse.xml');
        system('opensim-cmd run-tool Analyse.xml');
        %Cleanup
        delete Analyse.xml tempModel.osim
        clear AnTool FrAnalysis FwdTool

        %Load in states and force data
        statesD = importdata('_states.sto');
        forceD = importdata('_ForceReporter_forces.sto');

        %Grab out the shoulder rotation angle
        angInd = contains(statesD.colheaders,'shoulder_rot') & contains(statesD.colheaders,'value');
        optResults.(char(plication)).(rotMots{rr}).rotAngle = statesD.data(:,angInd);
        clear angInd statesD

        %Grab out the ligament force data
        forInd = contains(forceD.colheaders,'shoulder_rot_ligaments');
        optResults.(char(plication)).(rotMots{rr}).ligForce = forceD.data(:,forInd);
        clear forInd forceD

        %Cleanup files from simulationds
        delete _controls.sto _ForceReporter_forces.sto _states.sto _states_degrees.mot

        %Fita one term Gaussian model to the angle-force data
        %Found this model to perform best in matching the data points from the
        %forward simulations. Exponential curves didn't quite fit the curviness
        %as well as the Gaussian option
        optResults.(char(plication)).(rotMots{rr}).gaussFunc = ...
            fit(optResults.(char(plication)).(rotMots{rr}).rotAngle,optResults.(char(plication)).(rotMots{rr}).ligForce,'gauss1');
        %Plot and save exponential fit
        plot(optResults.(char(plication)).(rotMots{rr}).gaussFunc,optResults.(char(plication)).(rotMots{rr}).rotAngle,optResults.(char(plication)).(rotMots{rr}).ligForce);
        funcText = sprintf('f(x) = %s*exp(-((x-%s)/%s)^2)',...
            num2str(optResults.(char(plication)).(rotMots{rr}).gaussFunc.a1), num2str(optResults.(char(plication)).(rotMots{rr}).gaussFunc.b1),...
            num2str(optResults.(char(plication)).(rotMots{rr}).gaussFunc.c1));
        title({['Gaussian fit for ',(rotMots{rr}),' optimisation for "',plication,'" model.'],funcText});
        legend('Location','NorthEast');
        xlabel('External Shoulder Rotation (radians)'); ylabel('Ligament Force');
        h = gcf; set(h,'PaperPositionMode','auto');
        saveas(h,[rotMots{rr},'_GaussianFit_',plication,'Model.fig']);
        saveas(h,[rotMots{rr},'_GaussianFit_',plication,'Model.png']);
        close all; clear h funcText

        %Extract the coefficients from the curve fit
        optResults.(char(plication)).(rotMots{rr}).coeff.a = optResults.(char(plication)).(rotMots{rr}).gaussFunc.a1;
        optResults.(char(plication)).(rotMots{rr}).coeff.b = optResults.(char(plication)).(rotMots{rr}).gaussFunc.b1;
        optResults.(char(plication)).(rotMots{rr}).coeff.c = optResults.(char(plication)).(rotMots{rr}).gaussFunc.c1;

        %Cleanup
        clear x ans

        %Navigate back to optimisation directory
        cd('..');

    end
    clear rr rotMots

    %The next step is to model the 3 coefficients from the Gaussian models (a,
    %b and c) against shoulder elevation. This will therefore allow the
    %coefficients for the equations that produce the ligament force to be
    %modelled as a function of the two coordinates.

    %Create a new directory to store results
    mkdir('ExtRot_Elv'); cd('ExtRot_Elv');

    %Provide a variable for shoulder elevation angle
    sElv = [deg2rad(0);deg2rad(45);deg2rad(90)];

    %Extract coefficients into variables
    coeffA = [optResults.(char(plication)).ExtRot0.coeff.a
        optResults.(char(plication)).ExtRot45.coeff.a;
        optResults.(char(plication)).ExtRot90.coeff.a];
    coeffB = [optResults.(char(plication)).ExtRot0.coeff.b
        optResults.(char(plication)).ExtRot45.coeff.b;
        optResults.(char(plication)).ExtRot90.coeff.b];
    coeffC = [optResults.(char(plication)).ExtRot0.coeff.c
        optResults.(char(plication)).ExtRot45.coeff.c;
        optResults.(char(plication)).ExtRot90.coeff.c];

    %Based on an initial look at IR and None model results, the best fits are:
        % a, b & c = 2nd order (i.e. quadratic) polynomial
    %If this doesn't hold across different plication conditions then it would
    %be a matter of fitting a bunch of different equations and identifying the
    %one that had the least error/highest R-squared.

    %Fit quadratic polynomial curves to coefficients

    %Coeff a
    optResults.(char(plication)).ExtRot_Elv.aFunc = fit(sElv,coeffA,'poly2');
    %Plot and save results
    plot(optResults.(char(plication)).ExtRot_Elv.aFunc,sElv,coeffA);
    funcText = sprintf('f(x) = %s*x^2 + %s*x + %s',...
        num2str(optResults.(char(plication)).ExtRot_Elv.aFunc.p1), num2str(optResults.(char(plication)).ExtRot_Elv.aFunc.p2), num2str(optResults.(char(plication)).ExtRot_Elv.aFunc.p3));
    title({['Quadratic fit for ExtRot coefficient ''a'' for "',plication,'" model.'],funcText});
    legend('Location','NorthEast');
    xlabel('Shoulder Elevation (radians)'); ylabel('Function Value');
    h = gcf; set(h,'PaperPositionMode','auto');
    saveas(h,['ExtRot_coeffA_QuadraticFit_',plication,'Model.fig']);
    saveas(h,['ExtRot_coeffA_QuadraticFit_',plication,'Model.png']);
    close all; clear h funcText

    %Coeff b
    optResults.(char(plication)).ExtRot_Elv.bFunc = fit(sElv,coeffB,'poly2');
    %Plot and save results
    plot(optResults.(char(plication)).ExtRot_Elv.bFunc,sElv,coeffB);
    funcText = sprintf('f(x) = %s*x^2 + %s*x + %s',...
        num2str(optResults.(char(plication)).ExtRot_Elv.bFunc.p1), num2str(optResults.(char(plication)).ExtRot_Elv.bFunc.p2), num2str(optResults.(char(plication)).ExtRot_Elv.bFunc.p3));
    title({['Quadratic fit for ExtRot coefficient ''b'' for "',plication,'" model.'],funcText});
    legend('Location','NorthEast');
    xlabel('Shoulder Elevation (radians)'); ylabel('Function Value');
    h = gcf; set(h,'PaperPositionMode','auto');
    saveas(h,['ExtRot_coeffB_QuadraticFit_',plication,'Model.fig']);
    saveas(h,['ExtRot_coeffB_QuadraticFit_',plication,'Model.png']);
    close all; clear h funcText

    %Coeff c
    optResults.(char(plication)).ExtRot_Elv.cFunc = fit(sElv,coeffC,'poly2');
    %Plot and save results
    plot(optResults.(char(plication)).ExtRot_Elv.cFunc,sElv,coeffC);
    funcText = sprintf('f(x) = %s*x^2 + %s*x + %s',...
        num2str(optResults.(char(plication)).ExtRot_Elv.cFunc.p1), num2str(optResults.(char(plication)).ExtRot_Elv.cFunc.p2), num2str(optResults.(char(plication)).ExtRot_Elv.cFunc.p3));
    title({['Quadratic fit for ExtRot coefficient ''c'' for "',plication,'" model.'],funcText});
    legend('Location','NorthWest');
    xlabel('Shoulder Elevation (radians)'); ylabel('Function Value');
    h = gcf; set(h,'PaperPositionMode','auto');
    saveas(h,['ExtRot_coeffC_QuadraticFit_',plication,'Model.fig']);
    saveas(h,['ExtRot_coeffC_QuadraticFit_',plication,'Model.png']);
    close all; clear h funcText

    %We can now take the general one component Gaussian formula fit to each
    %of the internal rotation motions: a*exp(-((x-b)/c)^2); 
    %where x = internal rotation angle (i.e. q1), and substitute in the
    %functions for each of the coefficients, where x = shoulder elevation (i.e.
    %q2). This needs to have no white space for the opensim formatting.
    aTxt = sprintf('(%s*q2^2+%s*q2+%s)',...
        num2str(optResults.(char(plication)).ExtRot_Elv.aFunc.p1), num2str(optResults.(char(plication)).ExtRot_Elv.aFunc.p2), num2str(optResults.(char(plication)).ExtRot_Elv.aFunc.p3));
    bTxt = sprintf('(%s*q2^2+%s*q2+%s)',...
        num2str(optResults.(char(plication)).ExtRot_Elv.bFunc.p1), num2str(optResults.(char(plication)).ExtRot_Elv.bFunc.p2), num2str(optResults.(char(plication)).ExtRot_Elv.bFunc.p3));
    cTxt = sprintf('(%s*q2^2+%s*q2+%s)',...
        num2str(optResults.(char(plication)).ExtRot_Elv.cFunc.p1), num2str(optResults.(char(plication)).ExtRot_Elv.cFunc.p2), num2str(optResults.(char(plication)).ExtRot_Elv.cFunc.p3));
    coordForceFunc.(char(plication)).ExtRot = [aTxt,'*exp(-((q1-',bTxt,')/',cTxt,')^2)'];

    %Navigate back to optimisation directory
    cd('..');

    %Cleanup
    clear aTxt bTxt cTxt coeffA coeffB coeffC sElv

    %% Last do the elevation movements movements

    %Run simulations with each of the optimised rotation ligament parameters to
    %get the force-angle profiles out


    %Set a variable for rotation motion outputs
    elvMots = [{'Abduction'}; {'Flexion'}];

    %Loop through the motions
    for rr = 1:length(elvMots)

        %Navigate to optimisation directory for motion
        cd(elvMots{rr});

        %Set the optimised input parameters
        x = ligParamOpt.(char(plication)).(elvMots{rr}).xOpt;

        %Set parameters
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_elv_ligaments')).set_upper_limit(x(1));
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_elv_ligaments')).set_upper_stiffness(x(2));
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_elv_ligaments')).set_transition(x(3));

        %Run the forward simulation
        %Set the shoulder coordinate values
        if strcmp(elvMots{rr},'Abduction')
            %Set the shoulder coordinate values
            osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(30));
            osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(0));
            osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(0));
        elseif strcmp(elvMots{rr},'Flexion')
            %Set the shoulder coordinate values
            osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(90));
            osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(0));
            osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(0));        
        end

        %Set locking on shoulder coordinates
        osimModel.getCoordinateSet().get('elv_angle').set_locked(true);
        osimModel.getCoordinateSet().get('shoulder_elv').set_locked(false);
        osimModel.getCoordinateSet().get('shoulder_rot').set_locked(true);

        %Finalise model connections
        osimModel.finalizeConnections();

        %Initialise the forward tool
        FwdTool = ForwardTool();

        %Settings for forward tool
        FwdTool.setModel(osimModel);
        FwdTool.setInitialTime(0); FwdTool.setFinalTime(5);

        %Set controls for simulation
        FwdTool.setControlsFileName([modelDir,'\shoulder_elv_controls.xml']);

        %Need to add controller set for the controls to be active in the model
        %First, clear any existing controller set from past simulations
        osimModel.getControllerSet().clearAndDestroy();
        %Add the current controller set to the model
        FwdTool.addControllerSetToModel();

        %Run forward tool
        FwdTool.run();

        %Run a force reporter analysis tool on the forward simulation results
        %Printing of model and tool seems necessary as running from Matlab command
        %doesn't recognise states file added in without doing this
        AnTool = AnalyzeTool(); AnTool.setModelFilename('tempModel.osim');
        AnTool.setStatesFileName('_states.sto');
        AnTool.setStartTime(0); AnTool.setFinalTime(5);
        FrAnalysis = ForceReporter();
        FrAnalysis.setStartTime(0); FrAnalysis.setEndTime(5);
        AnTool.getAnalysisSet().cloneAndAppend(FrAnalysis);
        osimModel.print('tempModel.osim'); AnTool.print('Analyse.xml');
        system('opensim-cmd run-tool Analyse.xml');
        %Cleanup
        delete Analyse.xml tempModel.osim
        clear AnTool FrAnalysis FwdTool

        %Load in states and force data
        statesD = importdata('_states.sto');
        forceD = importdata('_ForceReporter_forces.sto');

        %Grab out the shoulder rotation angle
        angInd = contains(statesD.colheaders,'shoulder_elv') & contains(statesD.colheaders,'value');
        optResults.(char(plication)).(elvMots{rr}).elvAngle = statesD.data(:,angInd);
        clear angInd statesD

        %Grab out the ligament force data
        forInd = contains(forceD.colheaders,'shoulder_elv_ligaments');
        optResults.(char(plication)).(elvMots{rr}).ligForce = forceD.data(:,forInd);
        clear forInd forceD

        %Cleanup files from simulationds
        delete _controls.sto _ForceReporter_forces.sto _states.sto _states_degrees.mot

        %Fit a one term Gaussian model to the angle-force data
        %Found this model to perform best in matching the data points from the
        %forward simulations. Exponential curves didn't quite fit the curviness
        %as well as the Gaussian option
        optResults.(char(plication)).(elvMots{rr}).gaussFunc = ...
            fit(optResults.(char(plication)).(elvMots{rr}).elvAngle,optResults.(char(plication)).(elvMots{rr}).ligForce,'gauss1');
        %Plot and save exponential fit
        plot(optResults.(char(plication)).(elvMots{rr}).gaussFunc,optResults.(char(plication)).(elvMots{rr}).elvAngle,optResults.(char(plication)).(elvMots{rr}).ligForce);
        funcText = sprintf('f(x) = %s*exp(-((x-%s)/%s)^2)',...
            num2str(optResults.(char(plication)).(elvMots{rr}).gaussFunc.a1), num2str(optResults.(char(plication)).(elvMots{rr}).gaussFunc.b1),...
            num2str(optResults.(char(plication)).(elvMots{rr}).gaussFunc.c1));
        title({['Gaussian fit for ',(elvMots{rr}),' optimisation for "',plication,'" model.'],funcText});
        legend('Location','SouthWest');
        xlabel('Shoulder Elevation (radians)'); ylabel('Ligament Force');
        h = gcf; set(h,'PaperPositionMode','auto');
        saveas(h,[elvMots{rr},'_GaussianFit_',plication,'Model.fig']);
        saveas(h,[elvMots{rr},'_GaussianFit_',plication,'Model.png']);
        close all; clear h funcText

        %Extract the coefficients from the curve fit
        optResults.(char(plication)).(elvMots{rr}).coeff.a = optResults.(char(plication)).(elvMots{rr}).gaussFunc.a1;
        optResults.(char(plication)).(elvMots{rr}).coeff.b = optResults.(char(plication)).(elvMots{rr}).gaussFunc.b1;
        optResults.(char(plication)).(elvMots{rr}).coeff.c = optResults.(char(plication)).(elvMots{rr}).gaussFunc.c1;

        %Cleanup
        clear x ans

        %Navigate back to optimisation directory
        cd('..');

    end
    clear rr elvMots

    %The next step is to model the 3 coefficients from the Gaussian models (a,
    %b and c) against elevation angle. This will therefore allow the
    %coefficients for the equations that produce the ligament force to be
    %modelled as a function of the two coordinates.

    %Create a new directory to store results
    mkdir('Elv_ElvAng'); cd('Elv_ElvAng');

    %Provide a variable for shoulder elevation angle
    elvAng = [deg2rad(0);deg2rad(90)];

    %Extract coefficients into variables
    coeffA = [optResults.(char(plication)).Abduction.coeff.a
        optResults.(char(plication)).Flexion.coeff.a];
    coeffB = [optResults.(char(plication)).Abduction.coeff.b
        optResults.(char(plication)).Flexion.coeff.b];
    coeffC = [optResults.(char(plication)).Abduction.coeff.c
        optResults.(char(plication)).Flexion.coeff.c];

    %Given that we only have two elevation angle positions, the relationship
    %here can only really be modelled by a linear polynomial. It is therefore
    %likely that this relationship will have to hold across the different
    %plications.

    %Fit linear polynomial curves to coefficients

    %Coeff a
    optResults.(char(plication)).Elv_ElvAng.aFunc = fit(elvAng,coeffA,'poly1');
    %Plot and save results
    plot(optResults.(char(plication)).Elv_ElvAng.aFunc,elvAng,coeffA);
    funcText = sprintf('f(x) = %s*x + %s',...
        num2str(optResults.(char(plication)).Elv_ElvAng.aFunc.p1), num2str(optResults.(char(plication)).Elv_ElvAng.aFunc.p2));
    title({['Linear fit for Elevation coefficient ''a'' for "',plication,'" model.'],funcText});
    legend('Location','NorthWest');
    xlabel('Elevation Angle (radians)'); ylabel('Function Value');
    h = gcf; set(h,'PaperPositionMode','auto');
    saveas(h,['Elv_coeffA_QuadraticFit_',plication,'Model.fig']);
    saveas(h,['Elv_coeffA_QuadraticFit_',plication,'Model.png']);
    close all; clear h funcText

    %Coeff b
    optResults.(char(plication)).Elv_ElvAng.bFunc = fit(elvAng,coeffB,'poly1');
    %Plot and save results
    plot(optResults.(char(plication)).Elv_ElvAng.bFunc,elvAng,coeffB);
    funcText = sprintf('f(x) = %s*x + %s',...
        num2str(optResults.(char(plication)).Elv_ElvAng.bFunc.p1), num2str(optResults.(char(plication)).Elv_ElvAng.bFunc.p2));
    title({['Linear fit for Elevation coefficient ''b'' for "',plication,'" model.'],funcText});
    legend('Location','NorthEast');
    xlabel('Elevation Angle (radians)'); ylabel('Function Value');
    h = gcf; set(h,'PaperPositionMode','auto');
    saveas(h,['Elv_coeffB_QuadraticFit_',plication,'Model.fig']);
    saveas(h,['Elv_coeffB_QuadraticFit_',plication,'Model.png']);
    close all; clear h funcText

    %Coeff c
    optResults.(char(plication)).Elv_ElvAng.cFunc = fit(elvAng,coeffC,'poly1');
    %Plot and save results
    plot(optResults.(char(plication)).Elv_ElvAng.cFunc,elvAng,coeffC);
    funcText = sprintf('f(x) = %s*x + %s',...
        num2str(optResults.(char(plication)).Elv_ElvAng.cFunc.p1), num2str(optResults.(char(plication)).Elv_ElvAng.cFunc.p2));
    title({['Linear fit for Elevation coefficient ''b'' for "',plication,'" model.'],funcText});
    legend('Location','NorthEast');
    xlabel('Elevation Angle (radians)'); ylabel('Function Value');
    h = gcf; set(h,'PaperPositionMode','auto');
    saveas(h,['Elv_coeffC_QuadraticFit_',plication,'Model.fig']);
    saveas(h,['Elv_coeffC_QuadraticFit_',plication,'Model.png']);
    close all; clear h funcText

    %We can now take the general one component Gaussian formula fit to each
    %of the internal rotation motions: a*exp(-((x-b)/c)^2); 
    %where x = shoulder elevation angle (i.e. q1), and substitute in the
    %functions for each of the coefficients, where x = elevation angle (i.e.
    %q2). This needs to have no white space for the opensim formatting.
    aTxt = sprintf('(%s*q2+%s)',...
        num2str(optResults.(char(plication)).Elv_ElvAng.aFunc.p1), num2str(optResults.(char(plication)).Elv_ElvAng.aFunc.p2));
    bTxt = sprintf('(%s*q2+%s)',...
        num2str(optResults.(char(plication)).Elv_ElvAng.bFunc.p1), num2str(optResults.(char(plication)).Elv_ElvAng.bFunc.p2));
    cTxt = sprintf('(%s*q2+%s)',...
        num2str(optResults.(char(plication)).Elv_ElvAng.cFunc.p1), num2str(optResults.(char(plication)).Elv_ElvAng.cFunc.p2));
    coordForceFunc.(char(plication)).Elv = [aTxt,'*exp(-((q1-',bTxt,')/',cTxt,')^2)'];

    %Navigate back to optimisation directory
    cd('..');

    %Cleanup
    clear aTxt bTxt cTxt coeffA coeffB coeffC elvAng

    %% Now that the values for the 'None' plication have been established,
    %  the same process can be looped through for the additional
    %  plications. Given that all results (except 1) compared to the 'None'
    %  model are reduced range of motion - the optimisation can start at
    %  the 'None' values but also have these as the upper/lower bounds on
    %  the optimisations for limit and stiffness. We don't really want the
    %  transition parameter to change, but to keep the same optimisation
    %  routine we can just alow this parameter to fluctuate really
    %  slightly. The exception is that external rotation at 45 degress of
    %  abduction slightly increases with total posterior plication, so a
    %  checked statement needs to be included for this.
    
    %% Loop through each of the remaining plications
    for pp = 2:length(plications)
        
        %Set the curren plication
        plication = plications{pp};
        
        %% Loop through each of the angles/motions to optimise the CLF parameters
        for mm = 1:length(angles)

            %Set the motion variable for optimisations
            motion = angles{mm};

            %Setup inputs for optimisation function
            Input.plicationType = plication;
            Input.motionType = motion;
            Input.modelDir = modelDir;

            %Set the starting parameters for the bounds on optimisations.
            %Note that we'll use the results from the 'none' plication
            %results as starting points and upper/lower bounds for most
            %scenarios. This is a lot easier to code than the first time as
            %the variables are already stored in a similar fashion
            x0 = ligParamOpt.None.(char(motion)).xOpt;
            
            %Set the bounds. Only check here is whether to use the upper or
            %lower bounds for the limit variable. Upper bounds will be used
            %for internal rotation and elevation motions, while lower
            %bounds for external rotation motions. added some extra scope
            %for stiffness bounds to increase to potentially accomodate
            %some of the reduced joint range of motion.

            %First run the check for is it is the external rotation at 45
            %degrees of abduction under the total posterior condition. If
            %this is the case then we need to use the original bounds for
            %this motion as the range of motion does go slightly up.
            if strcmp(plication,'TotalPosterior') && strcmp(motion,'ExtRot45')
                %Use original bounds
                xUB = zeros(3,1); xLB = zeros(3,1);
                %Upper bounds
                xUB(1) = -1; xUB(2) = 250; xUB(3) = x0(3)+1e-08;
                %Lower bounds
                xLB(1) = -140; xLB(2) = 5; xLB(3) = x0(3)-1e-08;
            else
                %Apply appropriate bounds from original optimisation  
                if contains(motion,'ExtRot')
                    %Set the bounds
                    xUB = zeros(3,1); xLB = zeros(3,1);
                    %Upper bounds
                    xUB(1) = -0.1; xUB(2) = 500; xUB(3) = x0(3)+1e-08;
                    %Lower bounds
                    xLB(1) = x0(1); xLB(2) = x0(2); xLB(3) = x0(3)-1e-08;                
                elseif contains(motion,'IntRot')
                    %Set the bounds
                    xUB = zeros(3,1); xLB = zeros(3,1);
                    %Upper bounds
                    xUB(1) = x0(1); xUB(2) = 500; xUB(3) = x0(3)+1e-08;
                    %Lower bounds
                    xLB(1) = 0.1; xLB(2) = x0(1); xLB(3) = x0(3)-1e-08;
                else %elevation motions
                    %Set the bounds
                    xUB = zeros(3,1); xLB = zeros(3,1);
                    %Upper bounds
                    xUB(1) = x0(1); xUB(2) = 500; xUB(3) = x0(3)+1e-08;
                    %Lower bounds
                    xLB(1) = 0.1; xLB(2) = x0(1); xLB(3) = x0(3)-1e-08;                
                end
            end

            %% Run the optimisation

            %%%% TO DO: figure out how to supress constant outputs from opensim tools
            %%%% TO DO: potentially write optimisation steps to file

            %Set optimisation options
            options = optimset('fminsearch');
            options.PlotFcns = @optimplotfval;

            %Create folder for current motion optimisation
            cd(motion);

            %Start a timer for optimisation
            tStart = tic;

            %Run the optimisation
            [ligParamOpt.(char(plication)).(char(motion)).xOpt,...
                ligParamOpt.(char(plication)).(char(motion)).fval,...
                ligParamOpt.(char(plication)).(char(motion)).exitflag,...
                ligParamOpt.(char(plication)).(char(motion)).output] = ...
                fminsearchbnd('ligamentOptimiser',x0,xLB,xUB,options,Input);

            tEnd = toc(tStart);

            %Save fminsearch figure
            h = gcf;
            title({[motion,' optimisation for "',plication,'" model.'],...
                ['End function value: ',num2str(ligParamOpt.(char(plication)).(char(motion)).fval)]});
            set(h,'PaperPositionMode','auto')
            saveas(h,[motion,'_Optimisation_',plication,'Model.fig']);
            saveas(h,[motion,'_Optimisation_',plication,'Model.png']);
            close all

            %Export optimisation results as text file
            fid = fopen([motion,'_Optimisation_',plication,'Model_Results.txt'],'wt');
            fprintf(fid,['%d minutes and %f seconds on computer: ',getenv('COMPUTERNAME'),'\n'], floor(tEnd/60), rem(tEnd,60));
            if contains(motion,'IntRot')
                fprintf(fid,['Shoulder rotation ligament upper limit: ',num2str(ligParamOpt.(char(plication)).(char(motion)).xOpt(1)),'\n']);
                fprintf(fid,['Shoulder rotation upper stiffness: ',num2str(ligParamOpt.(char(plication)).(char(motion)).xOpt(2)),'\n']);
                fprintf(fid,['Shoulder rotation transition: ',num2str(ligParamOpt.(char(plication)).(char(motion)).xOpt(3))]);
            elseif contains(motion,'ExtRot')
                fprintf(fid,['Shoulder rotation ligament lower limit: ',num2str(ligParamOpt.(char(plication)).(char(motion)).xOpt(1)),'\n']);
                fprintf(fid,['Shoulder rotation lower stiffness: ',num2str(ligParamOpt.(char(plication)).(char(motion)).xOpt(2)),'\n']);
                fprintf(fid,['Shoulder rotation transition: ',num2str(ligParamOpt.(char(plication)).(char(motion)).xOpt(3))]);
            else
                fprintf(fid,['Shoulder elevation ligament upper limit: ',num2str(ligParamOpt.(char(plication)).(char(motion)).xOpt(1)),'\n']);
                fprintf(fid,['Shoulder elevation upper stiffness: ',num2str(ligParamOpt.(char(plication)).(char(motion)).xOpt(2)),'\n']);
                fprintf(fid,['Shoulder elevation transition: ',num2str(ligParamOpt.(char(plication)).(char(motion)).xOpt(3))]);
            end
            fclose(fid);

            %Cleanup
            clear h ax fid tStart tEnd

            %Exit out of directory
            cd('..');

            %Cleanup
            clear ans motion x0 xUB xLB

        end
        clear mm       
    end
    clear pp
    
    %Put a check in place that re-runs the optimisation with options that
    %allow the transition parameter to be modified to try and improve the
    %results if the end error is greater than 1 degree
    for pp = 2:length(plications)
        
        %Set the curren plication
        plication = plications{pp};
        
        %Loop through each of the angles/motions to optimise the CLF parameters
        for mm = 1:length(angles)

            %Only re-run optimisation if the original error was > 1
            if ligParamOpt.(plications{pp}).(angles{mm}).fval > 1

                %Set the motion variable for optimisations
                motion = angles{mm};

                %Setup inputs for optimisation function
                Input.plicationType = plication;
                Input.motionType = motion;
                Input.modelDir = modelDir;

                %Set the starting parameters for the bounds on optimisations.
                %Note that we'll use the results from the 'failed'
                %optimisation here
                x0 = ligParamOpt.(char(plication)).(char(motion)).xOpt;

                %Set the bounds. Only check here is whether to use the upper or
                %lower bounds for the limit variable. Upper bounds will be used
                %for internal rotation and elevation motions, while lower
                %bounds for external rotation motions. added some extra scope
                %for stiffness bounds to increase to potentially accomodate
                %some of the reduced joint range of motion.

                %First run the check for is it is the external rotation at 45
                %degrees of abduction under the total posterior condition. If
                %this is the case then we need to use the original bounds for
                %this motion as the range of motion does go slightly up.
                if strcmp(plication,'TotalPosterior') && strcmp(motion,'ExtRot45')
                    %Use original bounds
                    xUB = zeros(3,1); xLB = zeros(3,1);
                    %Upper bounds
                    xUB(1) = -1; xUB(2) = 250; xUB(3) = 700;
                    %Lower bounds
                    xLB(1) = -140; xLB(2) = 5; xLB(3) = 350;
                else
                    %Apply appropriate bounds from original optimisation  
                    if contains(motion,'ExtRot')
                        %Set the bounds
                        xUB = zeros(3,1); xLB = zeros(3,1);
                        %Upper bounds
                        xUB(1) = -0.1; xUB(2) = 500; xUB(3) = 700;
                        %Lower bounds
                        xLB(1) = x0(1); xLB(2) = x0(2); xLB(3) = 350;                
                    elseif contains(motion,'IntRot')
                        %Set the bounds
                        xUB = zeros(3,1); xLB = zeros(3,1);
                        %Upper bounds
                        xUB(1) = x0(1); xUB(2) = 500; xUB(3) = 700;
                        %Lower bounds
                        xLB(1) = 0.1; xLB(2) = x0(1); xLB(3) = 350;
                    else %elevation motions
                        %Set the bounds
                        xUB = zeros(3,1); xLB = zeros(3,1);
                        %Upper bounds
                        xUB(1) = x0(1); xUB(2) = 500; xUB(3) = 700;
                        %Lower bounds
                        xLB(1) = 0.1; xLB(2) = x0(1); xLB(3) = 350;                
                    end
                end

                %% Run the optimisation

                %%%% TO DO: figure out how to supress constant outputs from opensim tools
                %%%% TO DO: potentially write optimisation steps to file

                %Set optimisation options
                options = optimset('fminsearch');
                options.PlotFcns = @optimplotfval;

                %Create folder for current motion optimisation
                cd(motion);

                %Start a timer for optimisation
                tStart = tic;

                %Run the optimisation
                [ligParamOpt.(char(plication)).(char(motion)).xOpt,...
                    ligParamOpt.(char(plication)).(char(motion)).fval,...
                    ligParamOpt.(char(plication)).(char(motion)).exitflag,...
                    ligParamOpt.(char(plication)).(char(motion)).output] = ...
                    fminsearchbnd('ligamentOptimiser',x0,xLB,xUB,options,Input);

                tEnd = toc(tStart);

                %Save fminsearch figure
                h = gcf;
                title({[motion,' optimisation for "',plication,'" model.'],...
                    ['End function value: ',num2str(ligParamOpt.(char(plication)).(char(motion)).fval)]});
                set(h,'PaperPositionMode','auto')
                saveas(h,[motion,'_Optimisation_',plication,'Model.fig']);
                saveas(h,[motion,'_Optimisation_',plication,'Model.png']);
                close all

                %Export optimisation results as text file
                fid = fopen([motion,'_Optimisation_',plication,'Model_Results.txt'],'wt');
                fprintf(fid,'\nSecondary optimisation after first failed attempt:\n\n');
                fprintf(fid,['%d minutes and %f seconds on computer: ',getenv('COMPUTERNAME'),'\n'], floor(tEnd/60), rem(tEnd,60));
                if contains(motion,'IntRot')
                    fprintf(fid,['Shoulder rotation ligament upper limit: ',num2str(ligParamOpt.(char(plication)).(char(motion)).xOpt(1)),'\n']);
                    fprintf(fid,['Shoulder rotation upper stiffness: ',num2str(ligParamOpt.(char(plication)).(char(motion)).xOpt(2)),'\n']);
                    fprintf(fid,['Shoulder rotation transition: ',num2str(ligParamOpt.(char(plication)).(char(motion)).xOpt(3))]);
                elseif contains(motion,'ExtRot')
                    fprintf(fid,['Shoulder rotation ligament lower limit: ',num2str(ligParamOpt.(char(plication)).(char(motion)).xOpt(1)),'\n']);
                    fprintf(fid,['Shoulder rotation lower stiffness: ',num2str(ligParamOpt.(char(plication)).(char(motion)).xOpt(2)),'\n']);
                    fprintf(fid,['Shoulder rotation transition: ',num2str(ligParamOpt.(char(plication)).(char(motion)).xOpt(3))]);
                else
                    fprintf(fid,['Shoulder elevation ligament upper limit: ',num2str(ligParamOpt.(char(plication)).(char(motion)).xOpt(1)),'\n']);
                    fprintf(fid,['Shoulder elevation upper stiffness: ',num2str(ligParamOpt.(char(plication)).(char(motion)).xOpt(2)),'\n']);
                    fprintf(fid,['Shoulder elevation transition: ',num2str(ligParamOpt.(char(plication)).(char(motion)).xOpt(3))]);
                end
                fclose(fid);

                %Cleanup
                clear h ax fid tStart tEnd

                %Exit out of directory
                cd('..');

                %Cleanup
                clear ans motion x0 xUB xLB
                
            else
                %don't re-run optimisation
            end
            
        end
        clear mm
        
        %Cleanup
        clear plication
        
    end
    clear pp
    
    %% Use the optimisation parameters from corresponding motions to establish
    %  combined expressions that use the two coordinates relating to the
    %  specific motion (e.g. elevation and elevation angle; rotation and
    %  shoulder elevation) to develop a function that can be used in the dual
    %  expression based coordinate force.
    
    %Loop through for the edited plications
    for pp = 3:length(plications)
        
        %Set the curren plication
        plication = plications{pp};

        %% Start with the internal rotation movements

        %Run simulations with each of the optimised rotation ligament parameters to
        %get the force-angle profiles out

        %Set a variable for rotation motion outputs
        rotMots = [{'IntRot0'}; {'IntRot45'}; {'IntRot90'}];

        %Loop through the motions
        for rr = 1:length(rotMots)

            %Navigate to optimisation directory for motion
            cd(rotMots{rr});

            %Set the optimised input parameters
            x = ligParamOpt.(char(plication)).(rotMots{rr}).xOpt;

            %Set parameters
            CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_upper_limit(x(1));
            CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_upper_stiffness(x(2));
            CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_transition(x(3));

            %Run the forward simulation
            %Set the shoulder coordinate values
            if contains(rotMots{rr},'Rot0')
                %Set the shoulder coordinate values
                osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(30));
                osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(0));
                osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(0));
            elseif contains(rotMots{rr},'Rot45')
                %Set the shoulder coordinate values
                osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(30));
                osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(45));
                osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(0));        
            elseif contains(rotMots{rr},'Rot90')
                %Set the shoulder coordinate values
                osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(30));
                osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(90));
                osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(0)); 
            end

            %Set locking on shoulder coordinates
            osimModel.getCoordinateSet().get('elv_angle').set_locked(true);
            osimModel.getCoordinateSet().get('shoulder_elv').set_locked(true);
            osimModel.getCoordinateSet().get('shoulder_rot').set_locked(false);

            %Finalise model connections
            osimModel.finalizeConnections();

            %Initialise the forward tool
            FwdTool = ForwardTool();

            %Settings for forward tool
            FwdTool.setModel(osimModel);
            FwdTool.setInitialTime(0); FwdTool.setFinalTime(3);

            %Set controls for simulation
            FwdTool.setControlsFileName([modelDir,'\shoulder_introt_controls.xml']);

            %Need to add controller set for the controls to be active in the model
            %First, clear any existing controller set from past simulations
            osimModel.getControllerSet().clearAndDestroy();
            %Add the current controller set to the model
            FwdTool.addControllerSetToModel();

            %Run forward tool
            FwdTool.run();

            %Run a force reporter analysis tool on the forward simulation results
            %Printing of model and tool seems necessary as running from Matlab command
            %doesn't recognise states file added in without doing this
            AnTool = AnalyzeTool(); AnTool.setModelFilename('tempModel.osim');
            AnTool.setStatesFileName('_states.sto');
            AnTool.setStartTime(0); AnTool.setFinalTime(3);
            FrAnalysis = ForceReporter();
            FrAnalysis.setStartTime(0); FrAnalysis.setEndTime(3);
            AnTool.getAnalysisSet().cloneAndAppend(FrAnalysis);
            osimModel.print('tempModel.osim'); AnTool.print('Analyse.xml');
            system('opensim-cmd run-tool Analyse.xml');
            %Cleanup
            delete Analyse.xml tempModel.osim
            clear AnTool FrAnalysis FwdTool

            %Load in states and force data
            statesD = importdata('_states.sto');
            forceD = importdata('_ForceReporter_forces.sto');

            %Grab out the shoulder rotation angle
            angInd = contains(statesD.colheaders,'shoulder_rot') & contains(statesD.colheaders,'value');
            optResults.(char(plication)).(rotMots{rr}).rotAngle = statesD.data(:,angInd);
            clear angInd statesD

            %Grab out the ligament force data
            forInd = contains(forceD.colheaders,'shoulder_rot_ligaments');
            optResults.(char(plication)).(rotMots{rr}).ligForce = forceD.data(:,forInd);
            clear forInd forceD

            %Cleanup files from simulationds
            delete _controls.sto _ForceReporter_forces.sto _states.sto _states_degrees.mot

            %Fit a one term Gaussian model to the angle-force data
            %Found this model to perform best in matching the data points from the
            %forward simulations. Exponential curves didn't quite fit the curviness
            %as well as the Gaussian option
            optResults.(char(plication)).(rotMots{rr}).gaussFunc = ...
                fit(optResults.(char(plication)).(rotMots{rr}).rotAngle,optResults.(char(plication)).(rotMots{rr}).ligForce,'gauss1');
            %Plot and save exponential fit
            plot(optResults.(char(plication)).(rotMots{rr}).gaussFunc,optResults.(char(plication)).(rotMots{rr}).rotAngle,optResults.(char(plication)).(rotMots{rr}).ligForce);
            funcText = sprintf('f(x) = %s*exp(-((x-%s)/%s)^2)',...
                num2str(optResults.(char(plication)).(rotMots{rr}).gaussFunc.a1), num2str(optResults.(char(plication)).(rotMots{rr}).gaussFunc.b1),...
                num2str(optResults.(char(plication)).(rotMots{rr}).gaussFunc.c1));
            title({['Gaussian fit for ',(rotMots{rr}),' optimisation for "',plication,'" model.'],funcText});
            legend('Location','SouthWest');
            xlabel('Internal Shoulder Rotation (radians)'); ylabel('Ligament Force');
            h = gcf; set(h,'PaperPositionMode','auto');
            saveas(h,[rotMots{rr},'_GaussianFit_',plication,'Model.fig']);
            saveas(h,[rotMots{rr},'_GaussianFit_',plication,'Model.png']);
            close all; clear h funcText

            %Extract the coefficients from the curve fit
            optResults.(char(plication)).(rotMots{rr}).coeff.a = optResults.(char(plication)).(rotMots{rr}).gaussFunc.a1;
            optResults.(char(plication)).(rotMots{rr}).coeff.b = optResults.(char(plication)).(rotMots{rr}).gaussFunc.b1;
            optResults.(char(plication)).(rotMots{rr}).coeff.c = optResults.(char(plication)).(rotMots{rr}).gaussFunc.c1;

            %Cleanup
            clear x ans

            %Navigate back to optimisation directory
            cd('..');

        end
        clear rr rotMots

        %The next step is to model the 3 coefficients from the Gaussian models (a,
        %b and c) against shoulder elevation. This will therefore allow the
        %coefficients for the equations that produce the ligament force to be
        %modelled as a function of the two coordinates.

        %Create a new directory to store results
        cd('IntRot_Elv');

        %Provide a variable for shoulder elevation angle
        sElv = [deg2rad(0);deg2rad(45);deg2rad(90)];

        %Extract coefficients into variables
        coeffA = [optResults.(char(plication)).IntRot0.coeff.a
            optResults.(char(plication)).IntRot45.coeff.a;
            optResults.(char(plication)).IntRot90.coeff.a];
        coeffB = [optResults.(char(plication)).IntRot0.coeff.b
            optResults.(char(plication)).IntRot45.coeff.b;
            optResults.(char(plication)).IntRot90.coeff.b];
        coeffC = [optResults.(char(plication)).IntRot0.coeff.c
            optResults.(char(plication)).IntRot45.coeff.c;
            optResults.(char(plication)).IntRot90.coeff.c];

        %Based on an initial look at IR and None model results, the best fits are:
            % a, b & c = 2nd order (i.e. quadratic) polynomial
        %If this doesn't hold across different plication conditions then it would
        %be a matter of fitting a bunch of different equations and identifying the
        %one that had the least error/highest R-squared.

        %Fit quadratic polynomial curves to coefficients

        %Coeff a
        optResults.(char(plication)).IntRot_Elv.aFunc = fit(sElv,coeffA,'poly2');
        %Plot and save results
        plot(optResults.(char(plication)).IntRot_Elv.aFunc,sElv,coeffA);
        funcText = sprintf('f(x) = %s*x^2 + %s*x + %s',...
            num2str(optResults.(char(plication)).IntRot_Elv.aFunc.p1), num2str(optResults.(char(plication)).IntRot_Elv.aFunc.p2), num2str(optResults.(char(plication)).IntRot_Elv.aFunc.p3));
        title({['Quadratic fit for IntRot coefficient ''a'' for "',plication,'" model.'],funcText});
        legend('Location','NorthEast');
        xlabel('Shoulder Elevation (radians)'); ylabel('Function Value');
        h = gcf; set(h,'PaperPositionMode','auto');
        saveas(h,['IntRot_coeffA_QuadraticFit_',plication,'Model.fig']);
        saveas(h,['IntRot_coeffA_QuadraticFit_',plication,'Model.png']);
        close all; clear h funcText

        %Coeff b
        optResults.(char(plication)).IntRot_Elv.bFunc = fit(sElv,coeffB,'poly2');
        %Plot and save results
        plot(optResults.(char(plication)).IntRot_Elv.bFunc,sElv,coeffB);
        funcText = sprintf('f(x) = %s*x^2 + %s*x + %s',...
            num2str(optResults.(char(plication)).IntRot_Elv.bFunc.p1), num2str(optResults.(char(plication)).IntRot_Elv.bFunc.p2), num2str(optResults.(char(plication)).IntRot_Elv.bFunc.p3));
        title({['Quadratic fit for IntRot coefficient ''b'' for "',plication,'" model.'],funcText});
        legend('Location','NorthEast');
        xlabel('Shoulder Elevation (radians)'); ylabel('Function Value');
        h = gcf; set(h,'PaperPositionMode','auto');
        saveas(h,['IntRot_coeffB_QuadraticFit_',plication,'Model.fig']);
        saveas(h,['IntRot_coeffB_QuadraticFit_',plication,'Model.png']);
        close all; clear h funcText

        %Coeff c
        optResults.(char(plication)).IntRot_Elv.cFunc = fit(sElv,coeffC,'poly2');
        %Plot and save results
        plot(optResults.(char(plication)).IntRot_Elv.cFunc,sElv,coeffC);
        funcText = sprintf('f(x) = %s*x^2 + %s*x + %s',...
            num2str(optResults.(char(plication)).IntRot_Elv.cFunc.p1), num2str(optResults.(char(plication)).IntRot_Elv.cFunc.p2), num2str(optResults.(char(plication)).IntRot_Elv.cFunc.p3));
        title({['Quadratic fit for IntRot coefficient ''c'' for "',plication,'" model.'],funcText});
        legend('Location','NorthWest');
        xlabel('Shoulder Elevation (radians)'); ylabel('Function Value');
        h = gcf; set(h,'PaperPositionMode','auto');
        saveas(h,['IntRot_coeffC_QuadraticFit_',plication,'Model.fig']);
        saveas(h,['IntRot_coeffC_QuadraticFit_',plication,'Model.png']);
        close all; clear h funcText

        %We can now take the general one component Gaussian formula fit to each
        %of the internal rotation motions: a*exp(-((x-b)/c)^2); 
        %where x = internal rotation angle (i.e. q1), and substitute in the
        %functions for each of the coefficients, where x = shoulder elevation (i.e.
        %q2). This needs to have no white space for the opensim formatting.
        aTxt = sprintf('(%s*q2^2+%s*q2+%s)',...
            num2str(optResults.(char(plication)).IntRot_Elv.aFunc.p1), num2str(optResults.(char(plication)).IntRot_Elv.aFunc.p2), num2str(optResults.(char(plication)).IntRot_Elv.aFunc.p3));
        bTxt = sprintf('(%s*q2^2+%s*q2+%s)',...
            num2str(optResults.(char(plication)).IntRot_Elv.bFunc.p1), num2str(optResults.(char(plication)).IntRot_Elv.bFunc.p2), num2str(optResults.(char(plication)).IntRot_Elv.bFunc.p3));
        cTxt = sprintf('(%s*q2^2+%s*q2+%s)',...
            num2str(optResults.(char(plication)).IntRot_Elv.cFunc.p1), num2str(optResults.(char(plication)).IntRot_Elv.cFunc.p2), num2str(optResults.(char(plication)).IntRot_Elv.cFunc.p3));
        coordForceFunc.(char(plication)).IntRot = [aTxt,'*exp(-((q1-',bTxt,')/',cTxt,')^2)'];

        %Navigate back to optimisation directory
        cd('..');

        %Cleanup
        clear aTxt bTxt cTxt coeffA coeffB coeffC sElv

        %% Next do external rotation movements

        %Run simulations with each of the optimised rotation ligament parameters to
        %get the force-angle profiles out

        %Set a variable for rotation motion outputs
        rotMots = [{'ExtRot0'}; {'ExtRot45'}; {'ExtRot90'}];

        %Loop through the motions
        for rr = 1:length(rotMots)

            %Navigate to optimisation directory for motion
            cd(rotMots{rr});

            %Set the optimised input parameters
            x = ligParamOpt.(char(plication)).(rotMots{rr}).xOpt;

            %Set parameters
            CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_lower_limit(x(1));
            CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_lower_stiffness(x(2));
            CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_transition(x(3));

            %Run the forward simulation
            %Set the shoulder coordinate values
            if contains(rotMots{rr},'Rot0')
                %Set the shoulder coordinate values
                osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(30));
                osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(0));
                osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(0));
            elseif contains(rotMots{rr},'Rot45')
                %Set the shoulder coordinate values
                osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(30));
                osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(45));
                osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(0));        
            elseif contains(rotMots{rr},'Rot90')
                %Set the shoulder coordinate values
                osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(30));
                osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(90));
                osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(0)); 
            end

            %Set locking on shoulder coordinates
            osimModel.getCoordinateSet().get('elv_angle').set_locked(true);
            osimModel.getCoordinateSet().get('shoulder_elv').set_locked(true);
            osimModel.getCoordinateSet().get('shoulder_rot').set_locked(false);

            %Finalise model connections
            osimModel.finalizeConnections();

            %Initialise the forward tool
            FwdTool = ForwardTool();

            %Settings for forward tool
            FwdTool.setModel(osimModel);
            FwdTool.setInitialTime(0); FwdTool.setFinalTime(3);

            %Set controls for simulation
            FwdTool.setControlsFileName([modelDir,'\shoulder_extrot_controls.xml']);

            %Need to add controller set for the controls to be active in the model
            %First, clear any existing controller set from past simulations
            osimModel.getControllerSet().clearAndDestroy();
            %Add the current controller set to the model
            FwdTool.addControllerSetToModel();

            %Run forward tool
            FwdTool.run();

            %Run a force reporter analysis tool on the forward simulation results
            %Printing of model and tool seems necessary as running from Matlab command
            %doesn't recognise states file added in without doing this
            AnTool = AnalyzeTool(); AnTool.setModelFilename('tempModel.osim');
            AnTool.setStatesFileName('_states.sto');
            AnTool.setStartTime(0); AnTool.setFinalTime(3);
            FrAnalysis = ForceReporter();
            FrAnalysis.setStartTime(0); FrAnalysis.setEndTime(3);
            AnTool.getAnalysisSet().cloneAndAppend(FrAnalysis);
            osimModel.print('tempModel.osim'); AnTool.print('Analyse.xml');
            system('opensim-cmd run-tool Analyse.xml');
            %Cleanup
            delete Analyse.xml tempModel.osim
            clear AnTool FrAnalysis FwdTool

            %Load in states and force data
            statesD = importdata('_states.sto');
            forceD = importdata('_ForceReporter_forces.sto');

            %Grab out the shoulder rotation angle
            angInd = contains(statesD.colheaders,'shoulder_rot') & contains(statesD.colheaders,'value');
            optResults.(char(plication)).(rotMots{rr}).rotAngle = statesD.data(:,angInd);
            clear angInd statesD

            %Grab out the ligament force data
            forInd = contains(forceD.colheaders,'shoulder_rot_ligaments');
            optResults.(char(plication)).(rotMots{rr}).ligForce = forceD.data(:,forInd);
            clear forInd forceD

            %Cleanup files from simulationds
            delete _controls.sto _ForceReporter_forces.sto _states.sto _states_degrees.mot

            %Fita one term Gaussian model to the angle-force data
            %Found this model to perform best in matching the data points from the
            %forward simulations. Exponential curves didn't quite fit the curviness
            %as well as the Gaussian option
            optResults.(char(plication)).(rotMots{rr}).gaussFunc = ...
                fit(optResults.(char(plication)).(rotMots{rr}).rotAngle,optResults.(char(plication)).(rotMots{rr}).ligForce,'gauss1');
            %Plot and save exponential fit
            plot(optResults.(char(plication)).(rotMots{rr}).gaussFunc,optResults.(char(plication)).(rotMots{rr}).rotAngle,optResults.(char(plication)).(rotMots{rr}).ligForce);
            funcText = sprintf('f(x) = %s*exp(-((x-%s)/%s)^2)',...
                num2str(optResults.(char(plication)).(rotMots{rr}).gaussFunc.a1), num2str(optResults.(char(plication)).(rotMots{rr}).gaussFunc.b1),...
                num2str(optResults.(char(plication)).(rotMots{rr}).gaussFunc.c1));
            title({['Gaussian fit for ',(rotMots{rr}),' optimisation for "',plication,'" model.'],funcText});
            legend('Location','NorthEast');
            xlabel('External Shoulder Rotation (radians)'); ylabel('Ligament Force');
            h = gcf; set(h,'PaperPositionMode','auto');
            saveas(h,[rotMots{rr},'_GaussianFit_',plication,'Model.fig']);
            saveas(h,[rotMots{rr},'_GaussianFit_',plication,'Model.png']);
            close all; clear h funcText

            %Extract the coefficients from the curve fit
            optResults.(char(plication)).(rotMots{rr}).coeff.a = optResults.(char(plication)).(rotMots{rr}).gaussFunc.a1;
            optResults.(char(plication)).(rotMots{rr}).coeff.b = optResults.(char(plication)).(rotMots{rr}).gaussFunc.b1;
            optResults.(char(plication)).(rotMots{rr}).coeff.c = optResults.(char(plication)).(rotMots{rr}).gaussFunc.c1;

            %Cleanup
            clear x ans

            %Navigate back to optimisation directory
            cd('..');

        end
        clear rr rotMots

        %The next step is to model the 3 coefficients from the Gaussian models (a,
        %b and c) against shoulder elevation. This will therefore allow the
        %coefficients for the equations that produce the ligament force to be
        %modelled as a function of the two coordinates.

        %Create a new directory to store results
        cd('ExtRot_Elv');

        %Provide a variable for shoulder elevation angle
        sElv = [deg2rad(0);deg2rad(45);deg2rad(90)];

        %Extract coefficients into variables
        coeffA = [optResults.(char(plication)).ExtRot0.coeff.a
            optResults.(char(plication)).ExtRot45.coeff.a;
            optResults.(char(plication)).ExtRot90.coeff.a];
        coeffB = [optResults.(char(plication)).ExtRot0.coeff.b
            optResults.(char(plication)).ExtRot45.coeff.b;
            optResults.(char(plication)).ExtRot90.coeff.b];
        coeffC = [optResults.(char(plication)).ExtRot0.coeff.c
            optResults.(char(plication)).ExtRot45.coeff.c;
            optResults.(char(plication)).ExtRot90.coeff.c];

        %Based on an initial look at IR and None model results, the best fits are:
            % a, b & c = 2nd order (i.e. quadratic) polynomial
        %If this doesn't hold across different plication conditions then it would
        %be a matter of fitting a bunch of different equations and identifying the
        %one that had the least error/highest R-squared.

        %Fit quadratic polynomial curves to coefficients

        %Coeff a
        optResults.(char(plication)).ExtRot_Elv.aFunc = fit(sElv,coeffA,'poly2');
        %Plot and save results
        plot(optResults.(char(plication)).ExtRot_Elv.aFunc,sElv,coeffA);
        funcText = sprintf('f(x) = %s*x^2 + %s*x + %s',...
            num2str(optResults.(char(plication)).ExtRot_Elv.aFunc.p1), num2str(optResults.(char(plication)).ExtRot_Elv.aFunc.p2), num2str(optResults.(char(plication)).ExtRot_Elv.aFunc.p3));
        title({['Quadratic fit for ExtRot coefficient ''a'' for "',plication,'" model.'],funcText});
        legend('Location','NorthEast');
        xlabel('Shoulder Elevation (radians)'); ylabel('Function Value');
        h = gcf; set(h,'PaperPositionMode','auto');
        saveas(h,['ExtRot_coeffA_QuadraticFit_',plication,'Model.fig']);
        saveas(h,['ExtRot_coeffA_QuadraticFit_',plication,'Model.png']);
        close all; clear h funcText

        %Coeff b
        optResults.(char(plication)).ExtRot_Elv.bFunc = fit(sElv,coeffB,'poly2');
        %Plot and save results
        plot(optResults.(char(plication)).ExtRot_Elv.bFunc,sElv,coeffB);
        funcText = sprintf('f(x) = %s*x^2 + %s*x + %s',...
            num2str(optResults.(char(plication)).ExtRot_Elv.bFunc.p1), num2str(optResults.(char(plication)).ExtRot_Elv.bFunc.p2), num2str(optResults.(char(plication)).ExtRot_Elv.bFunc.p3));
        title({['Quadratic fit for ExtRot coefficient ''b'' for "',plication,'" model.'],funcText});
        legend('Location','NorthEast');
        xlabel('Shoulder Elevation (radians)'); ylabel('Function Value');
        h = gcf; set(h,'PaperPositionMode','auto');
        saveas(h,['ExtRot_coeffB_QuadraticFit_',plication,'Model.fig']);
        saveas(h,['ExtRot_coeffB_QuadraticFit_',plication,'Model.png']);
        close all; clear h funcText

        %Coeff c
        optResults.(char(plication)).ExtRot_Elv.cFunc = fit(sElv,coeffC,'poly2');
        %Plot and save results
        plot(optResults.(char(plication)).ExtRot_Elv.cFunc,sElv,coeffC);
        funcText = sprintf('f(x) = %s*x^2 + %s*x + %s',...
            num2str(optResults.(char(plication)).ExtRot_Elv.cFunc.p1), num2str(optResults.(char(plication)).ExtRot_Elv.cFunc.p2), num2str(optResults.(char(plication)).ExtRot_Elv.cFunc.p3));
        title({['Quadratic fit for ExtRot coefficient ''c'' for "',plication,'" model.'],funcText});
        legend('Location','NorthWest');
        xlabel('Shoulder Elevation (radians)'); ylabel('Function Value');
        h = gcf; set(h,'PaperPositionMode','auto');
        saveas(h,['ExtRot_coeffC_QuadraticFit_',plication,'Model.fig']);
        saveas(h,['ExtRot_coeffC_QuadraticFit_',plication,'Model.png']);
        close all; clear h funcText

        %We can now take the general one component Gaussian formula fit to each
        %of the internal rotation motions: a*exp(-((x-b)/c)^2); 
        %where x = internal rotation angle (i.e. q1), and substitute in the
        %functions for each of the coefficients, where x = shoulder elevation (i.e.
        %q2). This needs to have no white space for the opensim formatting.
        aTxt = sprintf('(%s*q2^2+%s*q2+%s)',...
            num2str(optResults.(char(plication)).ExtRot_Elv.aFunc.p1), num2str(optResults.(char(plication)).ExtRot_Elv.aFunc.p2), num2str(optResults.(char(plication)).ExtRot_Elv.aFunc.p3));
        bTxt = sprintf('(%s*q2^2+%s*q2+%s)',...
            num2str(optResults.(char(plication)).ExtRot_Elv.bFunc.p1), num2str(optResults.(char(plication)).ExtRot_Elv.bFunc.p2), num2str(optResults.(char(plication)).ExtRot_Elv.bFunc.p3));
        cTxt = sprintf('(%s*q2^2+%s*q2+%s)',...
            num2str(optResults.(char(plication)).ExtRot_Elv.cFunc.p1), num2str(optResults.(char(plication)).ExtRot_Elv.cFunc.p2), num2str(optResults.(char(plication)).ExtRot_Elv.cFunc.p3));
        coordForceFunc.(char(plication)).ExtRot = [aTxt,'*exp(-((q1-',bTxt,')/',cTxt,')^2)'];

        %Navigate back to optimisation directory
        cd('..');

        %Cleanup
        clear aTxt bTxt cTxt coeffA coeffB coeffC sElv

        %% Last do the elevation movements movements

        %Run simulations with each of the optimised rotation ligament parameters to
        %get the force-angle profiles out

        %Set a variable for rotation motion outputs
        elvMots = [{'Abduction'}; {'Flexion'}];

        %Loop through the motions
        for rr = 1:length(elvMots)

            %Navigate to optimisation directory for motion
            cd(elvMots{rr});

            %Set the optimised input parameters
            x = ligParamOpt.(char(plication)).(elvMots{rr}).xOpt;

            %Set parameters
            CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_elv_ligaments')).set_upper_limit(x(1));
            CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_elv_ligaments')).set_upper_stiffness(x(2));
            CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_elv_ligaments')).set_transition(x(3));

            %Run the forward simulation
            %Set the shoulder coordinate values
            if strcmp(elvMots{rr},'Abduction')
                %Set the shoulder coordinate values
                osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(30));
                osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(0));
                osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(0));
            elseif strcmp(elvMots{rr},'Flexion')
                %Set the shoulder coordinate values
                osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(90));
                osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(0));
                osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(0));        
            end

            %Set locking on shoulder coordinates
            osimModel.getCoordinateSet().get('elv_angle').set_locked(true);
            osimModel.getCoordinateSet().get('shoulder_elv').set_locked(false);
            osimModel.getCoordinateSet().get('shoulder_rot').set_locked(true);

            %Finalise model connections
            osimModel.finalizeConnections();

            %Initialise the forward tool
            FwdTool = ForwardTool();

            %Settings for forward tool
            FwdTool.setModel(osimModel);
            FwdTool.setInitialTime(0); FwdTool.setFinalTime(5);

            %Set controls for simulation
            FwdTool.setControlsFileName([modelDir,'\shoulder_elv_controls.xml']);

            %Need to add controller set for the controls to be active in the model
            %First, clear any existing controller set from past simulations
            osimModel.getControllerSet().clearAndDestroy();
            %Add the current controller set to the model
            FwdTool.addControllerSetToModel();

            %Run forward tool
            FwdTool.run();

            %Run a force reporter analysis tool on the forward simulation results
            %Printing of model and tool seems necessary as running from Matlab command
            %doesn't recognise states file added in without doing this
            AnTool = AnalyzeTool(); AnTool.setModelFilename('tempModel.osim');
            AnTool.setStatesFileName('_states.sto');
            AnTool.setStartTime(0); AnTool.setFinalTime(5);
            FrAnalysis = ForceReporter();
            FrAnalysis.setStartTime(0); FrAnalysis.setEndTime(5);
            AnTool.getAnalysisSet().cloneAndAppend(FrAnalysis);
            osimModel.print('tempModel.osim'); AnTool.print('Analyse.xml');
            system('opensim-cmd run-tool Analyse.xml');
            %Cleanup
            delete Analyse.xml tempModel.osim
            clear AnTool FrAnalysis FwdTool

            %Load in states and force data
            statesD = importdata('_states.sto');
            forceD = importdata('_ForceReporter_forces.sto');

            %Grab out the shoulder rotation angle
            angInd = contains(statesD.colheaders,'shoulder_elv') & contains(statesD.colheaders,'value');
            optResults.(char(plication)).(elvMots{rr}).elvAngle = statesD.data(:,angInd);
            clear angInd statesD

            %Grab out the ligament force data
            forInd = contains(forceD.colheaders,'shoulder_elv_ligaments');
            optResults.(char(plication)).(elvMots{rr}).ligForce = forceD.data(:,forInd);
            clear forInd forceD

            %Cleanup files from simulationds
            delete _controls.sto _ForceReporter_forces.sto _states.sto _states_degrees.mot

            %Fit a one term Gaussian model to the angle-force data
            %Found this model to perform best in matching the data points from the
            %forward simulations. Exponential curves didn't quite fit the curviness
            %as well as the Gaussian option
            optResults.(char(plication)).(elvMots{rr}).gaussFunc = ...
                fit(optResults.(char(plication)).(elvMots{rr}).elvAngle,optResults.(char(plication)).(elvMots{rr}).ligForce,'gauss1');
            %Plot and save exponential fit
            plot(optResults.(char(plication)).(elvMots{rr}).gaussFunc,optResults.(char(plication)).(elvMots{rr}).elvAngle,optResults.(char(plication)).(elvMots{rr}).ligForce);
            funcText = sprintf('f(x) = %s*exp(-((x-%s)/%s)^2)',...
                num2str(optResults.(char(plication)).(elvMots{rr}).gaussFunc.a1), num2str(optResults.(char(plication)).(elvMots{rr}).gaussFunc.b1),...
                num2str(optResults.(char(plication)).(elvMots{rr}).gaussFunc.c1));
            title({['Gaussian fit for ',(elvMots{rr}),' optimisation for "',plication,'" model.'],funcText});
            legend('Location','SouthWest');
            xlabel('Shoulder Elevation (radians)'); ylabel('Ligament Force');
            h = gcf; set(h,'PaperPositionMode','auto');
            saveas(h,[elvMots{rr},'_GaussianFit_',plication,'Model.fig']);
            saveas(h,[elvMots{rr},'_GaussianFit_',plication,'Model.png']);
            close all; clear h funcText

            %Extract the coefficients from the curve fit
            optResults.(char(plication)).(elvMots{rr}).coeff.a = optResults.(char(plication)).(elvMots{rr}).gaussFunc.a1;
            optResults.(char(plication)).(elvMots{rr}).coeff.b = optResults.(char(plication)).(elvMots{rr}).gaussFunc.b1;
            optResults.(char(plication)).(elvMots{rr}).coeff.c = optResults.(char(plication)).(elvMots{rr}).gaussFunc.c1;

            %Cleanup
            clear x ans

            %Navigate back to optimisation directory
            cd('..');

        end
        clear rr elvMots

        %The next step is to model the 3 coefficients from the Gaussian models (a,
        %b and c) against elevation angle. This will therefore allow the
        %coefficients for the equations that produce the ligament force to be
        %modelled as a function of the two coordinates.

        %Create a new directory to store results
        cd('Elv_ElvAng');

        %Provide a variable for shoulder elevation angle
        elvAng = [deg2rad(0);deg2rad(90)];

        %Extract coefficients into variables
        coeffA = [optResults.(char(plication)).Abduction.coeff.a
            optResults.(char(plication)).Flexion.coeff.a];
        coeffB = [optResults.(char(plication)).Abduction.coeff.b
            optResults.(char(plication)).Flexion.coeff.b];
        coeffC = [optResults.(char(plication)).Abduction.coeff.c
            optResults.(char(plication)).Flexion.coeff.c];

        %Given that we only have two elevation angle positions, the relationship
        %here can only really be modelled by a linear polynomial. It is therefore
        %likely that this relationship will have to hold across the different
        %plications.

        %Fit linear polynomial curves to coefficients

        %Coeff a
        optResults.(char(plication)).Elv_ElvAng.aFunc = fit(elvAng,coeffA,'poly1');
        %Plot and save results
        plot(optResults.(char(plication)).Elv_ElvAng.aFunc,elvAng,coeffA);
        funcText = sprintf('f(x) = %s*x + %s',...
            num2str(optResults.(char(plication)).Elv_ElvAng.aFunc.p1), num2str(optResults.(char(plication)).Elv_ElvAng.aFunc.p2));
        title({['Linear fit for Elevation coefficient ''a'' for "',plication,'" model.'],funcText});
        legend('Location','NorthWest');
        xlabel('Elevation Angle (radians)'); ylabel('Function Value');
        h = gcf; set(h,'PaperPositionMode','auto');
        saveas(h,['Elv_coeffA_QuadraticFit_',plication,'Model.fig']);
        saveas(h,['Elv_coeffA_QuadraticFit_',plication,'Model.png']);
        close all; clear h funcText

        %Coeff b
        optResults.(char(plication)).Elv_ElvAng.bFunc = fit(elvAng,coeffB,'poly1');
        %Plot and save results
        plot(optResults.(char(plication)).Elv_ElvAng.bFunc,elvAng,coeffB);
        funcText = sprintf('f(x) = %s*x + %s',...
            num2str(optResults.(char(plication)).Elv_ElvAng.bFunc.p1), num2str(optResults.(char(plication)).Elv_ElvAng.bFunc.p2));
        title({['Linear fit for Elevation coefficient ''b'' for "',plication,'" model.'],funcText});
        legend('Location','NorthEast');
        xlabel('Elevation Angle (radians)'); ylabel('Function Value');
        h = gcf; set(h,'PaperPositionMode','auto');
        saveas(h,['Elv_coeffB_QuadraticFit_',plication,'Model.fig']);
        saveas(h,['Elv_coeffB_QuadraticFit_',plication,'Model.png']);
        close all; clear h funcText

        %Coeff c
        optResults.(char(plication)).Elv_ElvAng.cFunc = fit(elvAng,coeffC,'poly1');
        %Plot and save results
        plot(optResults.(char(plication)).Elv_ElvAng.cFunc,elvAng,coeffC);
        funcText = sprintf('f(x) = %s*x + %s',...
            num2str(optResults.(char(plication)).Elv_ElvAng.cFunc.p1), num2str(optResults.(char(plication)).Elv_ElvAng.cFunc.p2));
        title({['Linear fit for Elevation coefficient ''b'' for "',plication,'" model.'],funcText});
        legend('Location','NorthEast');
        xlabel('Elevation Angle (radians)'); ylabel('Function Value');
        h = gcf; set(h,'PaperPositionMode','auto');
        saveas(h,['Elv_coeffC_QuadraticFit_',plication,'Model.fig']);
        saveas(h,['Elv_coeffC_QuadraticFit_',plication,'Model.png']);
        close all; clear h funcText

        %We can now take the general one component Gaussian formula fit to each
        %of the internal rotation motions: a*exp(-((x-b)/c)^2); 
        %where x = shoulder elevation angle (i.e. q1), and substitute in the
        %functions for each of the coefficients, where x = elevation angle (i.e.
        %q2). This needs to have no white space for the opensim formatting.
        aTxt = sprintf('(%s*q2+%s)',...
            num2str(optResults.(char(plication)).Elv_ElvAng.aFunc.p1), num2str(optResults.(char(plication)).Elv_ElvAng.aFunc.p2));
        bTxt = sprintf('(%s*q2+%s)',...
            num2str(optResults.(char(plication)).Elv_ElvAng.bFunc.p1), num2str(optResults.(char(plication)).Elv_ElvAng.bFunc.p2));
        cTxt = sprintf('(%s*q2+%s)',...
            num2str(optResults.(char(plication)).Elv_ElvAng.cFunc.p1), num2str(optResults.(char(plication)).Elv_ElvAng.cFunc.p2));
        coordForceFunc.(char(plication)).Elv = [aTxt,'*exp(-((q1-',bTxt,')/',cTxt,')^2)'];

        %Navigate back to optimisation directory
        cd('..');

        %Cleanup
        clear aTxt bTxt cTxt coeffA coeffB coeffC elvAng plication
        
    end
    clear pp

    %% Save results for use in later scripts
    save('OptimisedLigamentParameters.mat',...
        'angles','coordForceFunc','ligParamOpt','meanAngles','optResults','plications');
    
    
    
    Temp save point for optimisationr esults up to here. Note that
    %%%%% some other variables will need to be created within this script
    %%%%% to get back up to speed - but this .mat file will contain the
    %%%%% time consuming optimisation aspects.
    %%%%%
    %%%%% At this point most optimisations are going OK but just altering
    %%%%% limits and stiffness, however 3 (TotalInferior, TotalPosterior
    %%%%% and Posteroinferior) models for the IntRot90 condition are not
    %%%%% reaching the desired angles with quite low limits and high
    %%%%% stiffness options - may be the case that transition parameter
    %%%%% needs to be altered, and if this is the case then it needs to be
    %%%%% allowed to be altered for everything...
    save(['tempOptResultsStore_',date,'.mat'],'coordForceFunc','ligParamOpt','optResults');

end



%----- End of ShoulderCapsulorrhaphySims_2_OptimiseLigamentParameters.m -----%