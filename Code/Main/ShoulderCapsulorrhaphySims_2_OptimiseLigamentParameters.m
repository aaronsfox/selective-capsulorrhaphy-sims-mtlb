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
    motionNo = input('Input motion selection:\n[1] Elevation\n[2] Rotation\nEnter number: ');
    switch motionNo
        case 1
            motion = 'Elevation';
        case 2
            motion = 'Rotation';
    end
    %Check if appropriate number was input
    if isempty(motion)
        error('Incorrect number selected for motion type');
    else
        %everything's fine
    end
else
    %Check whether appropriate value is input
    if strcmpi(motion,'Elevation') || strcmpi(motion,'Rotation')
        %everything's fine
    else
        %throw error
        error('motion input must be case sensitive string of "Elevation" or "Rotation"');
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
        
    case 'Rotation'
        
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
        x0 = zeros(8,1);
        x0(1) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).get_upper_limit();
        x0(2) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).get_upper_stiffness();
        x0(3) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).get_lower_limit();
        x0(4) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).get_lower_stiffness();
        x0(5) = CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).get_transition();
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
        x0(6) = xmlTree.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).upper_limit;
        x0(7) = xmlTree.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).upper_stiffness;
        x0(8) = xmlTree.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).transition;
        %Cleanup
        clear xmlTree

        %Set the bounds on the input variables
        xUB = zeros(8,1); xLB = zeros(8,1);
        %Upper bounds
        xUB(1) = 110; xUB(2) = 200; xUB(3) = -1; xUB(4) = 200; xUB(5) = 600;
        xUB(6) = 180; xUB(7) = 200; xUB(8) = 600;
        %Lower bounds
        xLB(1) = 1; xLB(2) = 5; xLB(3) = -140; xLB(4) = 5; xLB(5) = 400;
        xLB(6) = 1; xLB(7) = 1; xLB(8) = 400;
        
        %% Run optimisation



        %Supress outputs?


        %Set options
        options = optimset('fminsearch');
        % % % options.MaxFunEvals = 5;
        options.PlotFcns = @optimplotfval;
        
        %% Run rotation related parameter optimisation

        %Navigate to optimisation results directory
        cd('OptimisationResults');

        %%%%% TO DO: cleanup outputs
        
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
        %%%%% (easily within 3...)

        [ligamentParameterOptimisation.(char(plication)).(char(motion)).xOpt,...
            ligamentParameterOptimisation.(char(plication)).(char(motion)).fval,...
            ligamentParameterOptimisation.(char(plication)).(char(motion)).exitflag,...
            ligamentParameterOptimisation.(char(plication)).(char(motion)).output] = ...
            fminsearchbnd('ligamentOptimiser',x0,xLB,xUB,options,Input);
        
end

%%
end
%----- End of ShoulderCapsulorrhaphySims_2_OptimiseLigamentParameters.m -----%