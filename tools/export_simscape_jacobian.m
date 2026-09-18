function result = export_simscape_jacobian(modelPath, outputPath)
%EXPORT_SIMSCAPE_JACOBIAN Log the source model's home-state J and J_bm.
% The source SLX is loaded and simulated without being saved or modified on disk.

if nargin < 1 || strlength(string(modelPath)) == 0
    modelPath = "E:\matlab2023b\workfile\matlab23b_workfiles\flexiv_rizon4s\simscape_of_flexiv\flexiv_rizon4s_kinematics_2025_5_14.slx";
end
if nargin < 2 || strlength(string(outputPath)) == 0
    projectRoot = fileparts(fileparts(mfilename("fullpath")));
    outputPath = fullfile(projectRoot, "diagnostics", "simscape_jacobian_home.mat");
end

modelPath = char(modelPath);
outputPath = char(outputPath);
assert(isfile(modelPath), "Source model does not exist: %s", modelPath);
outputDirectory = fileparts(outputPath);
if ~isfolder(outputDirectory)
    mkdir(outputDirectory);
end

[modelDirectory, modelName] = fileparts(modelPath);
originalDirectory = pwd;
directoryCleanup = onCleanup(@() cd(originalDirectory)); %#ok<NASGU>
cd(modelDirectory);
addpath(genpath(modelDirectory));
load_system(modelPath);
modelCleanup = onCleanup(@() close_system(modelName, 0)); %#ok<NASGU>

subsystem = modelName + "/THE_JACOBIAN_OF_SYSTEM";
portHandles = get_param(subsystem, "PortHandles");
names = ["J", "Pe", "r_a", "rc", "k", "J_bm"];
assert(numel(portHandles.Outport) == numel(names), ...
    "Expected six Jacobian subsystem outputs, found %d", numel(portHandles.Outport));
for index = 1:numel(names)
    set_param(portHandles.Outport(index), ...
        "DataLogging", "on", ...
        "DataLoggingNameMode", "Custom", ...
        "DataLoggingName", names(index));
end

simulationInput = Simulink.SimulationInput(modelName);
simulationInput = simulationInput.setModelParameter( ...
    "StopTime", "0.001", ...
    "SignalLogging", "on", ...
    "SignalLoggingName", "logsout", ...
    "ReturnWorkspaceOutputs", "on");
simulationOutput = sim(simulationInput);
logs = simulationOutput.logsout;
assert(~isempty(logs), "The Simscape run returned no signal log");

result = struct();
result.sourceModel = modelPath;
result.modelName = modelName;
result.generatedAt = char(datetime("now", "Format", "yyyy-MM-dd'T'HH:mm:ssXXX"));
for index = 1:numel(names)
    signal = logs.get(char(names(index)));
    assert(~isempty(signal), "Missing logged signal: %s", names(index));
    result.(char(names(index))) = last_sample(signal.Values);
end
result.time = double(logs.get("J").Values.Time(end));

save(outputPath, "result", "-v7");
fprintf("Wrote %s\n", outputPath);
fprintf("J size: %s; J_bm size: %s; t = %.9g s\n", ...
    mat2str(size(result.J)), mat2str(size(result.J_bm)), result.time);
end

function value = last_sample(values)
data = double(values.Data);
time = double(values.Time);
if isempty(time) || isscalar(data)
    value = squeeze(data);
    return;
end
timeDimension = find(size(data) == numel(time), 1, "last");
if isempty(timeDimension)
    value = squeeze(data);
    return;
end
subscripts = repmat({':'}, 1, ndims(data));
subscripts{timeDimension} = size(data, timeDimension);
value = squeeze(data(subscripts{:}));
end
