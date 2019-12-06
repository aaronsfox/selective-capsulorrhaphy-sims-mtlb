function ShoulderCapsulorrhaphySims_2_OptimiseLigamentParameters(motion,plication)

% @author: Aaron Fox
% Centre for Sport Research, Deakin University
% aaron.f@deakin.edu.au
% 
% This code serves to optimise the parameters of the coordinate limit force
% 'ligaments' so that the joint angles reached by the shoulder during
% passively generate movement align with those presented in Gereber et al.
% (2003).
% 
% TO DO: Notes on simulation time (see innerLevel_BushingOptimisation.m doc
% for notes on this plus other info...)
% 
% TO DO: add Gerber et al. references

%% Check inputs to function
    

%%%%% TO DO: probably don't need motion as an input as we'll do both...

%Check for motion type input
if nargin < 1
    %Prompt for motion
    motion = [];
    motionNo = input('Input motion selection:\n[1] Elevation\n[2] Rotation\n[3] Rotation-Elevation\nEnter number: ');
    switch motionNo
        case 1
            motion = 'Elevation';
        case 2
            motion = 'Rotation';
        case 3
            motion = 'RotationElevation';
    end
    %Check if appropriate number was input
    if isempty(motion)
        error('Incorrect number selected for motion type');
    else
        %everything's fine
    end
else
    %Check whether appropriate value is input
    if strcmpi(motion,'Elevation') || strcmpi(motion,'Rotation') || strcmpi(motion,'RotationElevation')
        %everything's fine
    else
        %throw error
        error('motion input must be case sensitive string of "Elevation" or "Rotation" or "RotationElevation"');
    end
end

%Check for plication input
if nargin < 2
    %Prompt for plication type
    plication = [];
    plicationNo = input('Input plication selection:\n[1] None\n[2] Anterosuperior\n[3] Anteroinferior\n[4] TotalAnterior\n[5] Posterosuperior\n[6] Posteroinferior\n[7] Total Posterior\n[8] Total Superior\n[9] Total Inferior\nEnter number: ');
    switch plicationNo
        case 1
            plication = 'None';
        case 2
            plication = 'Anterosuperior';
        case 3
            plication = 'Anteroinferior';
        case 4
            plication = 'TotalAnterior';
        case 5
            plication = 'Posterosuperior';
        case 6
            plication = 'Posteroinferior';
        case 7
            plication = 'TotalPosterior';
        case 8
            plication = 'TotalSuperior';
        case 9
            plication = 'TotalInferior';
    end
    %Check if appropriate number was input
    if isempty(plication)
        error('Incorrect number selected for plication type');
    else
        %everything's fine
    end
else
    %Check whether appropriate value is input
    if strcmpi(plication,'None') || strcmp(plication,'Anterosuperior') || ...
            strcmp(plication,'Anteroinferior') || strcmp(plication,'TotalAnterior') || ...
            strcmp(plication,'Posterosuperior') || strcmp(plication,'Posteroinferior') || ...
            strcmp(plication,'TotalPosterior') || strcmp(plication,'TotalSuperior') || ...
            strcmp(plication,'TotalInferior')
        %everything's fine
    else
        %throw error
        error('plication input must be case sensitive string of "None", "Anterosuperior", "Anteroinferior", "Total Anterior", "Posterosuperior", "Posteroinferior", "Total Posterior", "Total Superior" or "Total Inferior"');
    end
end

%% Set-up

import org.opensim.modeling.*

%Set main directory
mainDir = pwd;

%Add supplementary code folder to path
addpath('..\Supplementary');

%Load in custom CLF library
cd('..\..\Plugin_CustomCLF');
opensimCommon.LoadOpenSimLibraryExact([pwd,'\build\Release\osimCustomCoordinateLimitForcePlugin.dll']);
pluginPath = [pwd,'\build\Release\osimCustomCoordinateLimitForcePlugin.dll'];

%Navigate to model directory
cd('..\ModelFiles');
modelDir = pwd;
%Add geometry directory
ModelVisualizer.addDirToGeometrySearchPaths([pwd,'\Geometry']);

%Get model file
ModelFile = [pwd,'\BasicShoulderComplex_withForces.osim'];

%% Set-up the model for forward simulations within optimisation

%Load the model
global osimModel
osimModel = Model(ModelFile);

%%% first rotation motion for test
motion = 'IntRot0';

%Setup inputs for optimisation function
Input.plicationType = plication;
Input.motionType = motion;
Input.modelDir = modelDir;
Input.pluginPath = pluginPath;

%Set parameters and run optimisation based on motion
%%%%% TO DO: clean this up as it doesn't need to be split. Both motions
%%%%% need to be run...
switch motion
    
    case 'Elevation'

        %Set initial guess based on starting parameters in model

        %The variables input to the optimisation are:
        %[1] Elevation ligament upper limit
        %[2] Elevation ligament upper stiffness
        %[3] Elevation ligament transition
        %[4] Elevation by elevation plane ligament upper limit
        %[5] Elevation by elevation plane ligament lower limit
        %[6] Elevation by elevation plane ligament upper and lower stiffness
        %[7] Elevation by elevation plane ligament transition
        x0 = zeros(7,1);
        x0(1) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_elv_ligaments')).get_upper_limit();
        x0(2) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_elv_ligaments')).get_upper_stiffness();
        x0(3) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_elv_ligaments')).get_transition();
        %Can't access custom class details through API, so need to do so via XML
        [xmlTree] = xml_readOSIM(ModelFile);
        %Find relevant custom CLF index
        for c = 1:length(xmlTree.Model.ForceSet.objects.CustomCoordinateLimitForce)
            if strcmp(xmlTree.Model.ForceSet.objects.CustomCoordinateLimitForce(c).ATTRIBUTE.name,'shoulder_elv_by_elv_angle_ligaments')
                clfInd = c;
            else
            end
        end
        clear c
        %Extract properties
        x0(4) = xmlTree.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).upper_limit;
        x0(5) = xmlTree.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).lower_limit;
        x0(6) = xmlTree.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).upper_stiffness;
        x0(7) = xmlTree.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).transition;
        %Cleanup
        clear xmlTree

        %Set the bounds on the input variables
        xUB = zeros(7,1); xLB = zeros(7,1);
        %Upper bounds
        xUB(1) = 180; xUB(2) = 200; xUB(3) = 600; xUB(4) = 130;
        xUB(5) = -1; xUB(6) = 10; xUB(7) = 600;
        %Lower bounds
        xLB(1) = 45; xLB(2) = 50; xLB(3) = 400; xLB(4) = 1;
        xLB(5) = -90; xLB(6) = 1e-8; xLB(7) = 400;

        %% Run optimisation



        %Supress outputs?


        %Set options
        options = optimset('fminsearch');
        % % % options.MaxFunEvals = 5;
        options.PlotFcns = @optimplotfval;


        %% Run elevation related parameter optimisation

        %Navigate to optimisation results directory
        cd('OptimisationResults');

        %%%%% TO DO: cleanup outputs

        [ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt,...
            ligamentParameterOptimisation.(char(plication)).(char(motion)).fval,...
            ligamentParameterOptimisation.(char(plication)).(char(motion)).exitflag,...
            ligamentParameterOptimisation.(char(plication)).(char(motion)).output] = ...
            fminsearchbnd('ligamentOptimiser',x0,xLB,xUB,options,Input);

        % % % xOpt =
        % % % 
        % % %  53.5888
        % % %    91.0349
        % % %   541.5717
        % % %    43.4252
        % % %   -41.0418
        % % %     2.3356
        % % %   573.8713

        %Save fminsearch figure
        h = gcf;
        title({['Elevation optimisation for "',plication,'" model.'],...
            ['End function value: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).fval)]});
        set(h,'PaperPositionMode','auto')
        saveas(h,['ElevationOptimisation_',plication,'Model.fig']);
        saveas(h,['ElevationOptimisation_',plication,'Model.png']);
        close all

        %Export optimisation results as text file
        fid = fopen(['ElevationOptimisation_',plication,'Model_Results.txt'],'wt');
        fprintf(fid,['Elevation ligament upper limit: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(1)),'\n']);
        fprintf(fid,['Elevation ligament upper stiffness: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(2)),'\n']);
        fprintf(fid,['Elevation ligament transition: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(3)),'\n']);
        fprintf(fid,['Elevation by elevation plane ligament upper limit: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(4)),'\n']);
        fprintf(fid,['Elevation by elevation plane ligament lower limit: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(5)),'\n']);
        fprintf(fid,['Elevation by elevation plane ligament upper and lower stiffness: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(6)),'\n']);
        fprintf(fid,['Elevation by elevation plane ligament transition: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(7)),'\n']);
        fclose(fid);

        %Cleanup
        clear h ax fid
        
        %Exit out of directory
        cd('..');
        
    case 'IntRot0'
        
        %Set initial guess based on starting parameters in model

        %The variables input to the optimisation are:
        %[1] Shoulder rotation ligament upper limit
        %[2] Shoulder rotation ligament upper stiffness
        %[3] Shoulder rotation ligament lower limit
        %[4] Shoulder rotation ligament lower stiffness
        %[5] Shoulder rotation ligament transition
        %[6] Shoulder rotation by shoulder elevation ligament upper limit
        %[7] Shoulder rotation by shoulder elevation ligament upper stiffness
        %[8] Shoulder rotation by shoulder elevation transition
        %NOTE: optimising just the upper limit characteristics of the
        %shoulder rotation by elevation angle ligament appear to offer the
        %characteristics of altered int. and ext. rotation desired with
        %changes in elevation angle.
% % %         x0 = zeros(8,1);
% % %         x0 = zeros(5,1);
        x0 = zeros(3,1);
        x0(1) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).get_upper_limit();
        x0(2) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).get_upper_stiffness();
        x0(3) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).get_transition();
% % %         x0(4) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).get_lower_stiffness();
% % %         x0(5) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).get_transition();
% % %         %Can't access custom class details through API, so need to do so via XML
% % %         [xmlTree] = xml_readOSIM(ModelFile);
% % %         %Find relevant custom CLF index
% % %         for c = 1:length(xmlTree.Model.ForceSet.objects.CustomCoordinateLimitForce)
% % %             if strcmp(xmlTree.Model.ForceSet.objects.CustomCoordinateLimitForce(c).ATTRIBUTE.name,'shoulder_rot_by_shoulder_elv_ligaments')
% % %                 clfInd = c;
% % %             else
% % %             end
% % %         end
% % %         clear c
% % %         %Extract properties
% % %         x0(6) = xmlTree.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).upper_limit;
% % %         x0(7) = xmlTree.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).upper_stiffness;
% % %         x0(8) = xmlTree.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).transition;
% % %         %Cleanup
% % %         clear xmlTree

        %Set the bounds on the input variables
% % %         xUB = zeros(8,1); xLB = zeros(8,1);
% % %         xUB = zeros(5,1); xLB = zeros(5,1);
        xUB = zeros(3,1); xLB = zeros(3,1);
        %Upper bounds
        xUB(1) = 110; xUB(2) = 200; xUB(3) = 600; %%%xUB(4) = 200; xUB(5) = 600;
% % %         xUB(6) = 180; xUB(7) = 200; xUB(8) = 600;
        %Lower bounds
        xLB(1) = 1; xLB(2) = 5; xLB(3) = 400; %%%xLB(4) = 5; xLB(5) = 400;
% % %         xLB(6) = 1; xLB(7) = 1; xLB(8) = 400;
        
        %% Run optimisation



        %Supress outputs? Write optimisation results to text in optimiser script?


        %Set options
        options = optimset('fminsearch');
        % % % options.MaxFunEvals = 5;
        options.PlotFcns = @optimplotfval;
        
        %% Run rotation related parameter optimisation

        %Navigate to optimisation results directory
        cd('OptimisationResults');

        %%%%% TO DO: cleanup outputs
        
        %%%% Stuck with original for all rotation
        
        %%%%% Altered this and optimiser code to run neutral elevation
        %%%%% rotation parameters first, then will do elevation with
        %%%%% ligament with these initial parameters optimised.
                %%%%% Could also just run internal and external rotation
                %%%%% movements at neutral elevation separately -- although
                %%%%% the transition parameter is consistent across both...
        
        %%%%% It will be interesting to see the robustness of the
        %%%%% optimisation parameters for rotation. Manupulating the
        %%%%% parameters in the current fashion has the desired effect
        %%%%% directional effect with changes to ER and IR with elevation,
        %%%%% but whether or not just manipulating these parameters can
        %%%%% achieve the angles closely or whether the allowed bounds are
        %%%%% enough to do this will be the interesting aspect...
        
        %%%%% Could probe around in GUI to determine what good values might
        %%%%% be to get an idea of how it might work...
        
        %%%%% To speed up could drastically shorten the simulaion time, the
        %%%%% peak rotation is reached considerably earlier than 5 seconds
        %%%%% (easily within 3...) -- fixed to 3 seconds
        
        tStart = tic;

        [ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt,...
            ligamentParameterOptimisation.(char(plication)).(char(motion)).fval,...
            ligamentParameterOptimisation.(char(plication)).(char(motion)).exitflag,...
            ligamentParameterOptimisation.(char(plication)).(char(motion)).output] = ...
            fminsearchbnd('ligamentOptimiser',x0,xLB,xUB,options,Input);
        
        tEnd = toc(tStart);
        clc; fprintf('%d minutes and %f seconds\n', floor(tEnd/60), rem(tEnd,60));

        %%% For all rotation:
% % %    xOpt =
% % % 
    % % %    27.8203
    % % %   148.6179
    % % %   -53.8131
    % % %   124.9685
    % % %   459.7338
    % % %    28.8658
    % % %    86.3126
    % % %   546.3928

% % %   fVal =
% % % 
% % %       78.1250

        %%% For just neutral rotation:
% % %    xOpt =
% % % 
% % % 27.8197
% % % 141.8462
% % % -36.5184
% % % 139.2741
% % % 452.5421

% % %   fVal =
% % % 
% % %       5.6500e-05
        
        %Save fminsearch figure
        h = gcf;
        title({['Int. Rot. (0 degrees abd.) optimisation for "',plication,'" model.'],...
            ['End function value: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).fval)]});
        set(h,'PaperPositionMode','auto')
        saveas(h,['IntRot0_Optimisation_',plication,'Model.fig']);
        saveas(h,['IntRot0_Optimisation_',plication,'Model.png']);
        close all

        %Export optimisation results as text file
        fid = fopen(['IntRot0_Optimisation_',plication,'Model_Results.txt'],'wt');
        fprintf(fid,['Rotation ligament upper limit: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(1)),'\n']);
        fprintf(fid,['Rotation ligament upper stiffness: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(2)),'\n']);
% % %         fprintf(fid,['Rotation ligament lower limit: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(3)),'\n']);
% % %         fprintf(fid,['Rotation ligament lower stiffness: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(4)),'\n']);
        fprintf(fid,['Rotation ligament transition: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(3)),'\n']);
% % %         fprintf(fid,['Rotation by shoulder elevation ligament upper limit: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(6)),'\n']);
% % %         fprintf(fid,['Rotation by shoulder elevation ligament upper stiffness: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(7)),'\n']);
% % %         fprintf(fid,['Rotation by shoulder elevation ligament transition: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(8)),'\n']);
        fclose(fid);

        %Cleanup
        clear h ax fid tStart tEnd
        
        %Exit out of directory
        cd('..');
        
% % %         %Set the parameters of the opensim model to the optimised rotation
% % %         %values so that they are appropriate for the rotation elevation
% % %         %optimisation routine
% % %         xOpt = ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt;
% % %         CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_upper_limit(xOpt(1));
% % %         CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_upper_stiffness(xOpt(2));
% % %         CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_lower_limit(xOpt(3));
% % %         CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_lower_stiffness(xOpt(4));
% % %         CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_transition(xOpt(5));
% % %         osimModel.finalizeConnections(); clear xOpt
        
        %%%%% TO DO: need to consider editing of updating parameters after
        %%%%% adjusting the looping/case-approach of this function
        
    case 'IntRot45'
        
        %Set initial guess based on starting parameters in model

        %The variables input to the optimisation are:
        %[1] Shoulder rotation ligament upper limit
        %[2] Shoulder rotation ligament upper stiffness
        %[3] Shoulder rotation ligament lower limit
        %[4] Shoulder rotation ligament lower stiffness
        %[5] Shoulder rotation ligament transition
        %[6] Shoulder rotation by shoulder elevation ligament upper limit
        %[7] Shoulder rotation by shoulder elevation ligament upper stiffness
        %[8] Shoulder rotation by shoulder elevation transition
        %NOTE: optimising just the upper limit characteristics of the
        %shoulder rotation by elevation angle ligament appear to offer the
        %characteristics of altered int. and ext. rotation desired with
        %changes in elevation angle.
% % %         x0 = zeros(8,1);
% % %         x0 = zeros(5,1);

%%%%% repeating after original IntRot0 will use the optimisation results
%%%%% from the model, given they are the final results - which might be a
%%%%% good thing actually...

        x0 = zeros(3,1);
        x0(1) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).get_upper_limit();
        x0(2) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).get_upper_stiffness();
        x0(3) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).get_transition();
% % %         x0(4) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).get_lower_stiffness();
% % %         x0(5) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).get_transition();
% % %         %Can't access custom class details through API, so need to do so via XML
% % %         [xmlTree] = xml_readOSIM(ModelFile);
% % %         %Find relevant custom CLF index
% % %         for c = 1:length(xmlTree.Model.ForceSet.objects.CustomCoordinateLimitForce)
% % %             if strcmp(xmlTree.Model.ForceSet.objects.CustomCoordinateLimitForce(c).ATTRIBUTE.name,'shoulder_rot_by_shoulder_elv_ligaments')
% % %                 clfInd = c;
% % %             else
% % %             end
% % %         end
% % %         clear c
% % %         %Extract properties
% % %         x0(6) = xmlTree.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).upper_limit;
% % %         x0(7) = xmlTree.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).upper_stiffness;
% % %         x0(8) = xmlTree.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).transition;
% % %         %Cleanup
% % %         clear xmlTree

        %Set the bounds on the input variables
% % %         xUB = zeros(8,1); xLB = zeros(8,1);
% % %         xUB = zeros(5,1); xLB = zeros(5,1);
        xUB = zeros(3,1); xLB = zeros(3,1);
        %Upper bounds
        xUB(1) = 110; xUB(2) = 200; xUB(3) = 600; %%%xUB(4) = 200; xUB(5) = 600;
% % %         xUB(6) = 180; xUB(7) = 200; xUB(8) = 600;
        %Lower bounds
        xLB(1) = 1; xLB(2) = 5; xLB(3) = 400; %%%xLB(4) = 5; xLB(5) = 400;
% % %         xLB(6) = 1; xLB(7) = 1; xLB(8) = 400;
        
        %% Run optimisation



        %Supress outputs? Write optimisation results to text in optimiser script?


        %Set options
        options = optimset('fminsearch');
        % % % options.MaxFunEvals = 5;
        options.PlotFcns = @optimplotfval;
        
        %% Run rotation related parameter optimisation

        %Navigate to optimisation results directory
        cd('OptimisationResults');

        %%%%% TO DO: cleanup outputs
        
        tStart = tic;

        [ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt,...
            ligamentParameterOptimisation.(char(plication)).(char(motion)).fval,...
            ligamentParameterOptimisation.(char(plication)).(char(motion)).exitflag,...
            ligamentParameterOptimisation.(char(plication)).(char(motion)).output] = ...
            fminsearchbnd('ligamentOptimiser',x0,xLB,xUB,options,Input);
        
        tEnd = toc(tStart);
        clc; fprintf('%d minutes and %f seconds\n', floor(tEnd/60), rem(tEnd,60));
        
        %Save fminsearch figure
        h = gcf;
        title({['Int. Rot. (45 degrees abd.) optimisation for "',plication,'" model.'],...
            ['End function value: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).fval)]});
        set(h,'PaperPositionMode','auto')
        saveas(h,['IntRot45_Optimisation_',plication,'Model.fig']);
        saveas(h,['IntRot45_Optimisation_',plication,'Model.png']);
        close all

        %Export optimisation results as text file
        fid = fopen(['IntRot45_Optimisation_',plication,'Model_Results.txt'],'wt');
        fprintf(fid,['Rotation ligament upper limit: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(1)),'\n']);
        fprintf(fid,['Rotation ligament upper stiffness: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(2)),'\n']);
% % %         fprintf(fid,['Rotation ligament lower limit: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(3)),'\n']);
% % %         fprintf(fid,['Rotation ligament lower stiffness: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(4)),'\n']);
        fprintf(fid,['Rotation ligament transition: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(3)),'\n']);
% % %         fprintf(fid,['Rotation by shoulder elevation ligament upper limit: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(6)),'\n']);
% % %         fprintf(fid,['Rotation by shoulder elevation ligament upper stiffness: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(7)),'\n']);
% % %         fprintf(fid,['Rotation by shoulder elevation ligament transition: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(8)),'\n']);
        fclose(fid);

        %Cleanup
        clear h ax fid tStart tEnd
        
        %Exit out of directory
        cd('..');
        
% % %         %Set the parameters of the opensim model to the optimised rotation
% % %         %values so that they are appropriate for the rotation elevation
% % %         %optimisation routine
% % %         xOpt = ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt;
% % %         CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_upper_limit(xOpt(1));
% % %         CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_upper_stiffness(xOpt(2));
% % %         CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_lower_limit(xOpt(3));
% % %         CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_lower_stiffness(xOpt(4));
% % %         CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_transition(xOpt(5));
% % %         osimModel.finalizeConnections(); clear xOpt
        
        %%%%% TO DO: need to consider editing of updating parameters after
        %%%%% adjusting the looping/case-approach of this function
        
    case 'IntRot90'
        
        %Set initial guess based on starting parameters in model

        %The variables input to the optimisation are:
        %[1] Shoulder rotation ligament upper limit
        %[2] Shoulder rotation ligament upper stiffness
        %[3] Shoulder rotation ligament lower limit
        %[4] Shoulder rotation ligament lower stiffness
        %[5] Shoulder rotation ligament transition
        %[6] Shoulder rotation by shoulder elevation ligament upper limit
        %[7] Shoulder rotation by shoulder elevation ligament upper stiffness
        %[8] Shoulder rotation by shoulder elevation transition
        %NOTE: optimising just the upper limit characteristics of the
        %shoulder rotation by elevation angle ligament appear to offer the
        %characteristics of altered int. and ext. rotation desired with
        %changes in elevation angle.
% % %         x0 = zeros(8,1);
% % %         x0 = zeros(5,1);

%%%%% repeating after original IntRot45 will use the optimisation results
%%%%% from the model, given they are the final results - which might be a
%%%%% good thing actually...

        x0 = zeros(3,1);
        x0(1) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).get_upper_limit();
        x0(2) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).get_upper_stiffness();
        x0(3) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).get_transition();
% % %         x0(4) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).get_lower_stiffness();
% % %         x0(5) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).get_transition();
% % %         %Can't access custom class details through API, so need to do so via XML
% % %         [xmlTree] = xml_readOSIM(ModelFile);
% % %         %Find relevant custom CLF index
% % %         for c = 1:length(xmlTree.Model.ForceSet.objects.CustomCoordinateLimitForce)
% % %             if strcmp(xmlTree.Model.ForceSet.objects.CustomCoordinateLimitForce(c).ATTRIBUTE.name,'shoulder_rot_by_shoulder_elv_ligaments')
% % %                 clfInd = c;
% % %             else
% % %             end
% % %         end
% % %         clear c
% % %         %Extract properties
% % %         x0(6) = xmlTree.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).upper_limit;
% % %         x0(7) = xmlTree.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).upper_stiffness;
% % %         x0(8) = xmlTree.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).transition;
% % %         %Cleanup
% % %         clear xmlTree

        %Set the bounds on the input variables
% % %         xUB = zeros(8,1); xLB = zeros(8,1);
% % %         xUB = zeros(5,1); xLB = zeros(5,1);
        xUB = zeros(3,1); xLB = zeros(3,1);
        %Upper bounds
        xUB(1) = 110; xUB(2) = 200; xUB(3) = 600; %%%xUB(4) = 200; xUB(5) = 600;
% % %         xUB(6) = 180; xUB(7) = 200; xUB(8) = 600;
        %Lower bounds
        xLB(1) = 1; xLB(2) = 5; xLB(3) = 400; %%%xLB(4) = 5; xLB(5) = 400;
% % %         xLB(6) = 1; xLB(7) = 1; xLB(8) = 400;
        
        %% Run optimisation



        %Supress outputs? Write optimisation results to text in optimiser script?


        %Set options
        options = optimset('fminsearch');
        % % % options.MaxFunEvals = 5;
        options.PlotFcns = @optimplotfval;
        
        %% Run rotation related parameter optimisation

        %Navigate to optimisation results directory
        cd('OptimisationResults');

        %%%%% TO DO: cleanup outputs
        
        tStart = tic;

        [ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt,...
            ligamentParameterOptimisation.(char(plication)).(char(motion)).fval,...
            ligamentParameterOptimisation.(char(plication)).(char(motion)).exitflag,...
            ligamentParameterOptimisation.(char(plication)).(char(motion)).output] = ...
            fminsearchbnd('ligamentOptimiser',x0,xLB,xUB,options,Input);
        
        tEnd = toc(tStart);
        clc; fprintf('%d minutes and %f seconds\n', floor(tEnd/60), rem(tEnd,60));
        
        %Save fminsearch figure
        h = gcf;
        title({['Int. Rot. (90 degrees abd.) optimisation for "',plication,'" model.'],...
            ['End function value: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).fval)]});
        set(h,'PaperPositionMode','auto')
        saveas(h,['IntRot90_Optimisation_',plication,'Model.fig']);
        saveas(h,['IntRot90_Optimisation_',plication,'Model.png']);
        close all

        %Export optimisation results as text file
        fid = fopen(['IntRot90_Optimisation_',plication,'Model_Results.txt'],'wt');
        fprintf(fid,['Rotation ligament upper limit: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(1)),'\n']);
        fprintf(fid,['Rotation ligament upper stiffness: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(2)),'\n']);
% % %         fprintf(fid,['Rotation ligament lower limit: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(3)),'\n']);
% % %         fprintf(fid,['Rotation ligament lower stiffness: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(4)),'\n']);
        fprintf(fid,['Rotation ligament transition: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(3)),'\n']);
% % %         fprintf(fid,['Rotation by shoulder elevation ligament upper limit: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(6)),'\n']);
% % %         fprintf(fid,['Rotation by shoulder elevation ligament upper stiffness: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(7)),'\n']);
% % %         fprintf(fid,['Rotation by shoulder elevation ligament transition: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(8)),'\n']);
        fclose(fid);

        %Cleanup
        clear h ax fid tStart tEnd
        
        %Exit out of directory
        cd('..');
        
% % %         %Set the parameters of the opensim model to the optimised rotation
% % %         %values so that they are appropriate for the rotation elevation
% % %         %optimisation routine
% % %         xOpt = ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt;
% % %         CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_upper_limit(xOpt(1));
% % %         CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_upper_stiffness(xOpt(2));
% % %         CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_lower_limit(xOpt(3));
% % %         CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_lower_stiffness(xOpt(4));
% % %         CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_transition(xOpt(5));
% % %         osimModel.finalizeConnections(); clear xOpt
        
        %%%%% TO DO: need to consider editing of updating parameters after
        %%%%% adjusting the looping/case-approach of this function
        
    case 'RotationElevation'
        
        %Set initial guess based on starting parameters in model

        %The variables input to the optimisation are:
        %[1] Shoulder rotation by shoulder elevation ligament upper limit
        %[2] Shoulder rotation by shoulder elevation ligament upper stiffness
        %[3] Shoulder rotation by shoulder elevation transition
        %NOTE: optimising just the upper limit characteristics of the
        %shoulder rotation by elevation angle ligament appear to offer the
        %characteristics of altered int. and ext. rotation desired with
        %changes in elevation angle.
        x0 = zeros(3,1);
        %Can't access custom class details through API, so need to do so via XML
        [xmlTree] = xml_readOSIM(ModelFile);
        %Find relevant custom CLF index
        for c = 1:length(xmlTree.Model.ForceSet.objects.CustomCoordinateLimitForce)
            if strcmp(xmlTree.Model.ForceSet.objects.CustomCoordinateLimitForce(c).ATTRIBUTE.name,'shoulder_rot_by_shoulder_elv_ligaments')
                clfInd = c;
            else
            end
        end
        clear c
        %Extract properties
        x0(1) = xmlTree.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).upper_limit;
        x0(2) = xmlTree.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).upper_stiffness;
        x0(3) = xmlTree.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).transition;
        %Cleanup
        clear xmlTree

        %Set the bounds on the input variables
        xUB = zeros(3,1); xLB = zeros(3,1);
        %Upper bounds
        xUB(1) = 44.9; xUB(2) = 1.0; xUB(3) = 800;
        %Lower bounds
        xLB(1) = 0.1; xLB(2) = 0.01; xLB(3) = 400;
        
        %% Run optimisation



        %Supress outputs? Write optimisation results to text in optimiser script?


        %Set options
        options = optimset('fminsearch');
        % % % options.MaxFunEvals = 5;
        options.PlotFcns = @optimplotfval;
        
        %% Run rotation related parameter optimisation

        %Navigate to optimisation results directory
        cd('OptimisationResults');
        
        %%% Undertook a 'piggy back' from original optimisation that didn't
        %%% work that well to see if greater improvements can be made -----
        %%% doesn't get much better
        
        %%% Just try the internal rotation values
        
% % %         xOpt =
% % % 
% % %     3.3284
% % %     0.1313
% % %   487.2695
       
% % % fVal =
% % % 
% % %   130.7880

        [ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt,...
            ligamentParameterOptimisation.(char(plication)).(char(motion)).fval,...
            ligamentParameterOptimisation.(char(plication)).(char(motion)).exitflag,...
            ligamentParameterOptimisation.(char(plication)).(char(motion)).output] = ...
            fminsearchbnd('ligamentOptimiser',x0,xLB,xUB,options,Input);
       
        %Save fminsearch figure
        h = gcf;
        title({['Rotation-Elevation optimisation for "',plication,'" model.'],...
            ['End function value: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).fval)]});
        set(h,'PaperPositionMode','auto')
        saveas(h,['RotationElevationOptimisation_',plication,'Model.fig']);
        saveas(h,['RotationElevationOptimisation_',plication,'Model.png']);
        close all

% % %         %Export optimisation results as text file
% % %         fid = fopen(['RotationOptimisation_',plication,'Model_Results.txt'],'wt');
% % %         fprintf(fid,['Rotation ligament upper limit: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(1)),'\n']);
% % %         fprintf(fid,['Rotation ligament upper stiffness: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(2)),'\n']);
% % %         fprintf(fid,['Rotation ligament lower limit: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(3)),'\n']);
% % %         fprintf(fid,['Rotation ligament lower stiffness: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(4)),'\n']);
% % %         fprintf(fid,['Rotation ligament transition: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(5)),'\n']);
% % %         fprintf(fid,['Rotation by shoulder elevation ligament upper limit: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(6)),'\n']);
% % %         fprintf(fid,['Rotation by shoulder elevation ligament upper stiffness: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(7)),'\n']);
% % %         fprintf(fid,['Rotation by shoulder elevation ligament transition: ',num2str(ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt(8)),'\n']);
        fclose(fid);

        %Cleanup
        clear h ax fid
        
        %Exit out of directory
        cd('..');
        
end

%% Test out process of using the three internal rotation results to model
%  the exponential curves at different elevation positions to establish a
%  combined expression that uses both internal rotation and elevation
%  angles to produce force --- with the goal to implement into a dual
%  expression based coordinate force

%Load the model with forces
osimModel = Model('BasicShoulderComplex_withForces.osim');

%For testing (for now) destroy the models force set
osimModel.getForceSet().clearAndDestroy();

%Re-add the coordinate actuators

%Create shoulder elevation coordinate actuator
elvActu = CoordinateActuator();             %constructor with default properties
elvActu.set_coordinate('shoulder_elv');     %set coordinate
elvActu.setOptimalForce(1);                 %optimal force corresponds to the force we wish to apply later
elvActu.setName('shoulder_elv_actuator');   %set actuator name

%Create shoulder rotation coordinate actuator
rotActu = CoordinateActuator();             %constructor with default properties
rotActu.set_coordinate('shoulder_rot');     %set coordinate
rotActu.setOptimalForce(0.5);               %optimal force corresponds to the force we wish to apply later
rotActu.setName('shoulder_rot_actuator');   %set actuator name

%Add forces to model
osimModel.addForce(elvActu);
osimModel.addForce(rotActu);

%Re-add the shoulder rotation ligaments just for this test

%Create the shoulder rotation force CLF
rotLig = CoordinateLimitForce();             %constructor with default properties
rotLig.setName('shoulder_rot_ligaments');    %set name
rotLig.set_coordinate('shoulder_rot');       %set coordinate
rotLig.set_upper_stiffness(100);             %set upper stiffness value
rotLig.set_upper_limit(45);                  %set upper limit threshold
rotLig.set_lower_stiffness(100);             %set lower stiffness value
rotLig.set_lower_limit(-45);                 %set lower limit threshold
rotLig.set_damping(0);                       %set damping value
rotLig.set_transition(485.46600000000001);   %set transition value

%Create the shoulder rotation damping CLF
rotDamp = CoordinateLimitForce();            %constructor with default properties
rotDamp.setName('shoulder_rot_damping');     %set name
rotDamp.set_coordinate('shoulder_rot');      %set coordinate
rotDamp.set_upper_stiffness(1e-08);          %set upper stiffness value
rotDamp.set_upper_limit(190);                %set upper limit threshold
rotDamp.set_lower_stiffness(1e-08);          %set lower stiffness value
rotDamp.set_lower_limit(190);                %set lower limit threshold
rotDamp.set_damping(0.001745);               %set damping value
rotDamp.set_transition(1);                   %set transition value

%Add forces to model
osimModel.addForce(rotLig)
osimModel.addForce(rotDamp)

%Run simulations with each of the optimised rotation ligament parameters to
%get the force-angle profiles out

%Set a variable for rotation motion outputs
rotMots = [{'IntRot0'}; {'IntRot45'}; {'IntRot90'}];

%Navigate to optimisation results
cd('OptimisationResults');

%Loop through the motions
for rr = 1:length(rotMots)
    
    %Set the optimised input parameters
    x = ligamentParameterOptimisation.(char(plication)).(rotMots{rr}).xOpt;

    %Set parameters
    CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_upper_limit(x(1));
    CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_upper_stiffness(x(2));
    CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_transition(x(3));

    %Run the forward simulation
    %Set the shoulder coordinate values
    
    %%%%%% TO DO: could actually set the rotation value appropriately here
    
    osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(30));
    osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(0));
    osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(0));

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
    %doesn't recognise states
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
    optimisedResults.(rotMots{rr}).rotAngle = statesD.data(:,angInd);
    clear angInd statesD

    %Grab out the ligament force data
    forInd = contains(forceD.colheaders,'shoulder_rot_ligaments');
    optimisedResults.(rotMots{rr}).ligForce = forceD.data(:,forInd);
    clear forInd forceD

    %Cleanup files from simulationds
    delete _controls.sto _ForceReporter_forces.sto _states.sto _states_degrees.mot

    %Fit an exponential curve to the angle-force data
    %The exponential function fits the output of coordinate limit forces well
    %A two term model seems to fit the early part of the data better
    optimisedResults.(rotMots{rr}).gaussFunc = ...
        fit(optimisedResults.(rotMots{rr}).rotAngle,optimisedResults.(rotMots{rr}).ligForce,'gauss1');
    %Plot and save exponential fit
    plot(optimisedResults.(rotMots{rr}).gaussFunc,optimisedResults.(rotMots{rr}).rotAngle,optimisedResults.(rotMots{rr}).ligForce)
    funcText = sprintf('f(x) = %s*exp(-((x-%s)/%s)^2)',...
        num2str(optimisedResults.(rotMots{rr}).gaussFunc.a1), num2str(optimisedResults.(rotMots{rr}).gaussFunc.b1),...
        num2str(optimisedResults.(rotMots{rr}).gaussFunc.c1));
    title({['Gaussian fit for ',(rotMots{rr}),' optimisation for "',plication,'" model.'],funcText});
    legend('Location','SouthWest');
    h = gcf; set(h,'PaperPositionMode','auto');
    saveas(h,[rotMots{rr},'_GaussianFit_',plication,'Model.fig']);
    saveas(h,[rotMots{rr},'_GaussianFit_',plication,'Model.png']);
    close all; clear h funcText
    
    %Extract the coefficients from the curve fit
    optimisedResults.(rotMots{rr}).coeff.a1 = optimisedResults.(rotMots{rr}).gaussFunc.a1;
    optimisedResults.(rotMots{rr}).coeff.b1 = optimisedResults.(rotMots{rr}).gaussFunc.b1;
    optimisedResults.(rotMots{rr}).coeff.c1 = optimisedResults.(rotMots{rr}).gaussFunc.c1;
% % %     optimisedResults.(rotMots{rr}).coeff.d = optimisedResults.(rotMots{rr}).expFunc.d;

    %Cleanup
    clear x ans
    
end
clear rr

%The next step is to model the 4 coefficients (a, b, c, d) of the
%exponential functions against shoulder elevation. This will therefore
%allow the coefficients for the equations that produce rotational
%resistance force to be modelled both as a function of rotation angle and
%shoulder elevation angle

%Provide a variable for shoulder elevation angle
sElv = [deg2rad(0);deg2rad(45);deg2rad(90)];
% % % sElv_xtra = [0;45;90;60];

%Extract coefficients into variables
coeffA1 = [optimisedResults.IntRot0.coeff.a1
    optimisedResults.IntRot45.coeff.a1;
    optimisedResults.IntRot90.coeff.a1];
coeffB1 = [optimisedResults.IntRot0.coeff.b1
    optimisedResults.IntRot45.coeff.b1;
    optimisedResults.IntRot90.coeff.b1];
coeffC1 = [optimisedResults.IntRot0.coeff.c1
    optimisedResults.IntRot45.coeff.c1;
    optimisedResults.IntRot90.coeff.c1];
% % % coeffD = [optimisedResults.IntRot0.coeff.d
% % %     optimisedResults.IntRot45.coeff.d;
% % %     optimisedResults.IntRot90.coeff.d];

% % % %Based on an initial look at IR and None model results, the best fits are:
% % %     % a & c = 3rd order (cubic polynomial) --> after establishing appropriate mid values with exponential fit with Levenberg-Marquardt algorithm
% % %     % b & d = 2nd order (i.e. quadratic) polynomial
% % % %If this doesn't hold across different plication conditions then it would
% % % %be a matter of fitting a bunch of different equations and identifying the
% % % %one that had the least error/highest R-squared.

%Based on an initial look at IR and None model results, the best fits are:
    % a1, b1 & c1 = 2nd order (i.e. quadratic) polynomial
%If this doesn't hold across different plication conditions then it would
%be a matter of fitting a bunch of different equations and identifying the
%one that had the least error/highest R-squared.
    
%Fit curves to coefficients

% % % %Coeff a
% % % fitOpts = fitoptions('exp1'); fitOpts.Algorithm = 'Levenberg-Marquardt'; %first fit exponential function
% % % fitOpts.Lower = -Inf; fitOpts.Upper = Inf; fitOpts.StartPoint = [0,0]; %first fit exponential function
% % % exp_aFunc = fit(sElv,coeffA,'exp1',fitOpts);    %first fit exponential function
% % % coeffA(end+1) = feval(exp_aFunc,60); %evaluatate exponential function at 60 degrees
% % % % % % plot(exp_aFunc,sElv_xtra,coeffA);
% % % %Now fit the polynomial function with extra data
% % % optimisedResults.IntRot_Elv.aFunc = fit(sElv_xtra,coeffA,'poly3');
% % % %Plot and save results
% % % plot(optimisedResults.IntRot_Elv.aFunc,sElv,coeffA(1:end-1));
% % % funcText = sprintf('f(x) = %s*x^3 + %s*x^2 + %s*x + %s',...
% % %     num2str(optimisedResults.IntRot_Elv.aFunc.p1), num2str(optimisedResults.IntRot_Elv.aFunc.p2),...
% % %     num2str(optimisedResults.IntRot_Elv.aFunc.p3), num2str(optimisedResults.IntRot_Elv.aFunc.p4));
% % % title({['Cubic polynomial fit for IntRot coefficient ''a'' for "',plication,'" model.'],funcText});
% % % legend('Location','NorthEast');
% % % h = gcf; set(h,'PaperPositionMode','auto');
% % % saveas(h,['IntRot_coeffA_CubicFit_',plication,'Model.fig']);
% % % saveas(h,['IntRot_coeffA_CubicFit_',plication,'Model.png']);
% % % close all; clear h funcText exp_aFunc

% % % %Coeff b
% % % optimisedResults.IntRot_Elv.bFunc = fit(sElv,coeffB,'poly2');
% % % %Plot and save results
% % % plot(optimisedResults.IntRot_Elv.bFunc,sElv,coeffB);
% % % funcText = sprintf('f(x) = %s*x^2 + %s*x + %s',...
% % %     num2str(optimisedResults.IntRot_Elv.bFunc.p1), num2str(optimisedResults.IntRot_Elv.bFunc.p2), num2str(optimisedResults.IntRot_Elv.bFunc.p3));
% % % title({['Quadratic fit for IntRot coefficient ''b'' for "',plication,'" model.'],funcText});
% % % legend('Location','NorthWest');
% % % h = gcf; set(h,'PaperPositionMode','auto');
% % % saveas(h,['IntRot_coeffB_QuadraticFit_',plication,'Model.fig']);
% % % saveas(h,['IntRot_coeffB_QuadraticFit_',plication,'Model.png']);
% % % close all; clear h funcText

% % % %Coeff c
% % % exp_cFunc = fit(sElv,coeffC,'exp1',fitOpts);    %first fit exponential function
% % % coeffC(end+1) = feval(exp_cFunc,60); %evaluatate exponential function at 60 degrees
% % % % % % plot(exp_cFunc,sElv_xtra,coeffC);
% % % %Now fit the polynomial function with extra data
% % % optimisedResults.IntRot_Elv.cFunc = fit(sElv_xtra,coeffC,'poly3');
% % % %Plot and save results
% % % plot(optimisedResults.IntRot_Elv.cFunc,sElv,coeffC(1:end-1));
% % % funcText = sprintf('f(x) = %s*x^3 + %s*x^2 + %s*x + %s',...
% % %     num2str(optimisedResults.IntRot_Elv.cFunc.p1), num2str(optimisedResults.IntRot_Elv.cFunc.p2),...
% % %     num2str(optimisedResults.IntRot_Elv.cFunc.p3), num2str(optimisedResults.IntRot_Elv.cFunc.p4));
% % % title({['Cubic polynomial fit for IntRot coefficient ''c'' for "',plication,'" model.'],funcText});
% % % legend('Location','NorthWest');
% % % h = gcf; set(h,'PaperPositionMode','auto');
% % % saveas(h,['IntRot_coeffC_CubicFit_',plication,'Model.fig']);
% % % saveas(h,['IntRot_coeffC_CubicFit_',plication,'Model.png']);
% % % close all; clear h funcText exp_cFunc

% % % %Coeff d
% % % optimisedResults.IntRot_Elv.dFunc = fit(sElv,coeffD,'poly2');
% % % %Plot and save results
% % % plot(optimisedResults.IntRot_Elv.dFunc,sElv,coeffD);
% % % funcText = sprintf('f(x) = %s*x^2 + %s*x + %s',...
% % %     num2str(optimisedResults.IntRot_Elv.dFunc.p1), num2str(optimisedResults.IntRot_Elv.dFunc.p2), num2str(optimisedResults.IntRot_Elv.dFunc.p3));
% % % title({['Quadratic fit for IntRot coefficient ''d'' for "',plication,'" model.'],funcText});
% % % legend('Location','NorthWest');
% % % h = gcf; set(h,'PaperPositionMode','auto');
% % % saveas(h,['IntRot_coeffD_QuadraticFit_',plication,'Model.fig']);
% % % saveas(h,['IntRot_coeffD_QuadraticFit_',plication,'Model.png']);
% % % close all; clear h funcText

%Coeff a1
optimisedResults.IntRot_Elv.a1Func = fit(sElv,coeffA1,'poly2');
%Plot and save results
plot(optimisedResults.IntRot_Elv.a1Func,sElv,coeffA1);
funcText = sprintf('f(x) = %s*x^2 + %s*x + %s',...
    num2str(optimisedResults.IntRot_Elv.a1Func.p1), num2str(optimisedResults.IntRot_Elv.a1Func.p2), num2str(optimisedResults.IntRot_Elv.a1Func.p3));
title({['Quadratic fit for IntRot coefficient ''a1'' for "',plication,'" model.'],funcText});
legend('Location','NorthEast');
h = gcf; set(h,'PaperPositionMode','auto');
saveas(h,['IntRot_coeffA1_QuadraticFit_',plication,'Model.fig']);
saveas(h,['IntRot_coeffA1_QuadraticFit_',plication,'Model.png']);
close all; clear h funcText

%Coeff b1
optimisedResults.IntRot_Elv.b1Func = fit(sElv,coeffB1,'poly2');
%Plot and save results
plot(optimisedResults.IntRot_Elv.b1Func,sElv,coeffB1);
funcText = sprintf('f(x) = %s*x^2 + %s*x + %s',...
    num2str(optimisedResults.IntRot_Elv.b1Func.p1), num2str(optimisedResults.IntRot_Elv.b1Func.p2), num2str(optimisedResults.IntRot_Elv.b1Func.p3));
title({['Quadratic fit for IntRot coefficient ''b1'' for "',plication,'" model.'],funcText});
legend('Location','NorthEast');
h = gcf; set(h,'PaperPositionMode','auto');
saveas(h,['IntRot_coeffB1_QuadraticFit_',plication,'Model.fig']);
saveas(h,['IntRot_coeffB1_QuadraticFit_',plication,'Model.png']);
close all; clear h funcText

%Coeff c1
optimisedResults.IntRot_Elv.c1Func = fit(sElv,coeffC1,'poly2');
%Plot and save results
plot(optimisedResults.IntRot_Elv.c1Func,sElv,coeffC1);
funcText = sprintf('f(x) = %s*x^2 + %s*x + %s',...
    num2str(optimisedResults.IntRot_Elv.c1Func.p1), num2str(optimisedResults.IntRot_Elv.c1Func.p2), num2str(optimisedResults.IntRot_Elv.c1Func.p3));
title({['Quadratic fit for IntRot coefficient ''c1'' for "',plication,'" model.'],funcText});
legend('Location','NorthEast');
h = gcf; set(h,'PaperPositionMode','auto');
saveas(h,['IntRot_coeffC1_QuadraticFit_',plication,'Model.fig']);
saveas(h,['IntRot_coeffC1_QuadraticFit_',plication,'Model.png']);
close all; clear h funcText


%We can now take the general two component exponential formula fit to each
%of the internal rotation motions: a*exp(b*x) + c*exp(d*x); 
%where x = internal rotation angle (i.e. q1), and substitute in the
%functions for each of the coefficients, where x = shoulder elevation (i.e.
%q2). This needs to have no white space for the opensim formatting.
% % % aTxt = sprintf('(%s*(q2*(acos(-1)/180))^3+%s*(q2*(acos(-1)/180))^2+%s*(q2*(acos(-1)/180))+%s)',...
% % %     num2str(optimisedResults.IntRot_Elv.aFunc.p1), num2str(optimisedResults.IntRot_Elv.aFunc.p2), num2str(optimisedResults.IntRot_Elv.aFunc.p3), num2str(optimisedResults.IntRot_Elv.aFunc.p4));
% % % bTxt = sprintf('(%s*(q2*(acos(-1)/180))^2+%s*(q2*(acos(-1)/180))+%s)',num2str(optimisedResults.IntRot_Elv.bFunc.p1), num2str(optimisedResults.IntRot_Elv.bFunc.p2), num2str(optimisedResults.IntRot_Elv.bFunc.p3));
% % % cTxt = sprintf('(%s*(q2*(acos(-1)/180))^3+%s*(q2*(acos(-1)/180))^2+%s*(q2*(acos(-1)/180))+%s)',...
% % %     num2str(optimisedResults.IntRot_Elv.cFunc.p1), num2str(optimisedResults.IntRot_Elv.cFunc.p2), num2str(optimisedResults.IntRot_Elv.cFunc.p3), num2str(optimisedResults.IntRot_Elv.cFunc.p4));
% % % dTxt = sprintf('(%s*(q2*(acos(-1)/180))^2+%s*(q2*(acos(-1)/180))+%s)',num2str(optimisedResults.IntRot_Elv.dFunc.p1), num2str(optimisedResults.IntRot_Elv.dFunc.p2), num2str(optimisedResults.IntRot_Elv.dFunc.p3));
% % % coordForceFunc.IntRot = [aTxt,'*exp(',bTxt,'*q1)+',cTxt,'*exp(',dTxt,'*q1)'];

a1Txt = sprintf('(%s*q2^2+%s*q2+%s)',...
    num2str(optimisedResults.IntRot_Elv.a1Func.p1), num2str(optimisedResults.IntRot_Elv.a1Func.p2), num2str(optimisedResults.IntRot_Elv.a1Func.p3));
b1Txt = sprintf('(%s*q2^2+%s*q2+%s)',...
    num2str(optimisedResults.IntRot_Elv.b1Func.p1), num2str(optimisedResults.IntRot_Elv.b1Func.p2), num2str(optimisedResults.IntRot_Elv.b1Func.p3));
c1Txt = sprintf('(%s*q2^2+%s*q2+%s)',...
    num2str(optimisedResults.IntRot_Elv.c1Func.p1), num2str(optimisedResults.IntRot_Elv.c1Func.p2), num2str(optimisedResults.IntRot_Elv.c1Func.p3));
coordForceFunc.IntRot = [a1Txt,'*exp(-((q1-',b1Txt,')/',c1Txt,')^2)'];

%%%%% TO DO: need to include plication label in above structures



%%
end
%----- End of ShoulderCapsulorrhaphySims_2_OptimiseLigamentParameters.m -----%