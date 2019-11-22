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
            plication = 'Total Posterior';
        case 8
            plication = 'Total Superior';
        case 9
            plication = 'Total Inferior';
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
            strcmp(plication,'Anteroinferior') || strcmp(plication,'Total Anterior') || ...
            strcmp(plication,'Posterosuperior') || strcmp(plication,'Posteroinferior') || ...
            strcmp(plication,'Total Posterior') || strcmp(plication,'Total Superior') || ...
            strcmp(plication,'Total Inferior')
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

%Set initial guess based on starting parameters in model

%%%%% TO DO: use optimised iterations of model if available???

%%%%% TO DO: below is only relevant to elevation motion

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

%%%%% TO DO: only relevant to  elevation motion at the moment

%Supress outputs?


%Set options
options = optimset('fminsearch');
% % % options.MaxFunEvals = 5;
options.PlotFcns = @optimplotfval;


%%% cleanup outputs

[xOpt,fval,exitflag,output] = fminsearchbnd('ligamentOptimiser',x0,xLB,xUB,options,Input)


%%%% Save figure too...



%%
end
%----- End of ShoulderCapsulorrhaphySims_2_OptimiseLigamentParameters.m -----%