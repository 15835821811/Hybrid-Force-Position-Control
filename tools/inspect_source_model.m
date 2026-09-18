function inventory = inspect_source_model(modelPath, outputDir)
%INSPECT_SOURCE_MODEL Export a machine-readable inventory of the source SLX.

if nargin < 1 || strlength(string(modelPath)) == 0
    modelPath = "E:\matlab2023b\workfile\matlab23b_workfiles\flexiv_rizon4s\simscape_of_flexiv\flexiv_rizon4s_kinematics_2025_5_14.slx";
end
if nargin < 2 || strlength(string(outputDir)) == 0
    outputDir = fullfile(fileparts(fileparts(mfilename("fullpath"))), "diagnostics");
end

modelPath = char(modelPath);
outputDir = char(outputDir);
assert(isfile(modelPath), "Source model does not exist: %s", modelPath);
if ~isfolder(outputDir)
    mkdir(outputDir);
end

[~, modelName] = fileparts(modelPath);
load_system(modelPath);
cleanup = onCleanup(@() close_system(modelName, 0)); %#ok<NASGU>

inventory = struct();
inventory.generatedAt = char(datetime("now", "Format", "yyyy-MM-dd'T'HH:mm:ssXXX"));
inventory.sourceModel = modelPath;
inventory.modelName = modelName;
inventory.configuration = struct( ...
    "solver", get_param(modelName, "Solver"), ...
    "solverType", get_param(modelName, "SolverType"), ...
    "fixedStep", get_param(modelName, "FixedStep"), ...
    "startTime", get_param(modelName, "StartTime"), ...
    "stopTime", get_param(modelName, "StopTime"), ...
    "simulationMode", get_param(modelName, "SimulationMode"));

callbackNames = ["PreLoadFcn", "PostLoadFcn", "InitFcn", "StartFcn", "StopFcn", "CloseFcn"];
callbacks = struct();
for index = 1:numel(callbackNames)
    name = callbackNames(index);
    callbacks.(name) = get_param(modelName, name);
end
inventory.callbacks = callbacks;

allBlocks = find_system(modelName, "FollowLinks", "on", ...
    "LookUnderMasks", "all", "Type", "Block");
blockRecords = repmat(struct( ...
    "path", "", "name", "", "parent", "", "blockType", "", ...
    "maskType", "", "referenceBlock", "", "ports", [], ...
    "commented", "", "sampleTime", ""), numel(allBlocks), 1);
for index = 1:numel(allBlocks)
    block = allBlocks{index};
    blockRecords(index).path = block;
    blockRecords(index).name = get_param(block, "Name");
    blockRecords(index).parent = get_param(block, "Parent");
    blockRecords(index).blockType = get_param(block, "BlockType");
    blockRecords(index).maskType = safe_get_param(block, "MaskType");
    blockRecords(index).referenceBlock = safe_get_param(block, "ReferenceBlock");
    blockRecords(index).ports = get_param(block, "Ports");
    blockRecords(index).commented = safe_get_param(block, "Commented");
    blockRecords(index).sampleTime = safe_get_param(block, "SampleTime");
end
inventory.blocks = blockRecords;
inventory.blockCount = numel(blockRecords);

blockTypes = string({blockRecords.blockType});
uniqueTypes = unique(blockTypes);
typeCounts = repmat(struct("blockType", "", "count", 0), numel(uniqueTypes), 1);
for index = 1:numel(uniqueTypes)
    typeCounts(index).blockType = char(uniqueTypes(index));
    typeCounts(index).count = nnz(blockTypes == uniqueTypes(index));
end
inventory.blockTypeCounts = typeCounts;

topBlocks = find_system(modelName, "SearchDepth", 1, "Type", "Block");
inventory.topLevelBlocks = topBlocks(:);
inventory.topLevelConnections = top_level_connections(modelName);

workspaceBlocks = find_system(modelName, "FollowLinks", "on", ...
    "LookUnderMasks", "all", "BlockType", "ToWorkspace");
workspaceRecords = repmat(struct("path", "", "variableName", "", ...
    "saveFormat", "", "sampleTime", ""), numel(workspaceBlocks), 1);
for index = 1:numel(workspaceBlocks)
    block = workspaceBlocks{index};
    workspaceRecords(index).path = block;
    workspaceRecords(index).variableName = get_param(block, "VariableName");
    workspaceRecords(index).saveFormat = get_param(block, "SaveFormat");
    workspaceRecords(index).sampleTime = get_param(block, "SampleTime");
end
inventory.toWorkspaceBlocks = workspaceRecords;

inventory.stateflowCharts = stateflow_charts(modelName);

try
    variables = Simulink.findVars(modelName, "SearchMethod", "cached");
    variableRecords = repmat(struct("name", "", "sourceType", "", "source", ""), numel(variables), 1);
    for index = 1:numel(variables)
        variableRecords(index).name = variables(index).Name;
        variableRecords(index).sourceType = variables(index).SourceType;
        variableRecords(index).source = variables(index).Source;
    end
    inventory.variables = variableRecords;
catch exception
    inventory.variables = struct("inspectionError", exception.message);
end

jsonPath = fullfile(outputDir, "source_model_inventory.json");
write_text(jsonPath, jsonencode(inventory, "PrettyPrint", true));

summaryPath = fullfile(outputDir, "source_model_summary.txt");
summary = compose_summary(inventory);
write_text(summaryPath, summary);

fprintf("Wrote %s\n", jsonPath);
fprintf("Wrote %s\n", summaryPath);
end

function value = safe_get_param(block, parameter)
try
    value = get_param(block, parameter);
catch
    value = "";
end
end

function records = top_level_connections(modelName)
lines = find_system(modelName, "FindAll", "on", "SearchDepth", 1, "Type", "line");
records = repmat(struct("source", "", "sourcePort", 0, ...
    "destination", "", "destinationPort", 0), 0, 1);
for lineIndex = 1:numel(lines)
    line = lines(lineIndex);
    sourceHandle = get_param(line, "SrcBlockHandle");
    if isempty(sourceHandle) || sourceHandle < 0
        continue;
    end
    destinationHandles = get_param(line, "DstBlockHandle");
    sourcePortHandles = get_param(line, "SrcPortHandle");
    destinationPortHandles = get_param(line, "DstPortHandle");
    sourcePort = port_number(sourcePortHandles, 1);
    for destinationIndex = 1:numel(destinationHandles)
        if destinationHandles(destinationIndex) < 0
            continue;
        end
        record = struct( ...
            "source", getfullname(sourceHandle), ...
            "sourcePort", sourcePort, ...
            "destination", getfullname(destinationHandles(destinationIndex)), ...
            "destinationPort", port_number(destinationPortHandles, destinationIndex));
        records(end + 1, 1) = record; %#ok<AGROW>
    end
end
end

function number = port_number(handles, index)
number = 0;
if isempty(handles) || index > numel(handles) || handles(index) < 0
    return;
end
try
    value = get_param(handles(index), "PortNumber");
    if isnumeric(value)
        number = double(value);
    else
        number = str2double(value);
    end
    if isnan(number)
        number = 0;
    end
catch
    number = 0;
end
end

function records = stateflow_charts(modelName)
root = sfroot;
charts = root.find("-isa", "Stateflow.EMChart");
records = repmat(struct("path", "", "name", "", "script", ""), 0, 1);
for index = 1:numel(charts)
    chart = charts(index);
    if strcmp(chart.Path, modelName) || startsWith(chart.Path, [modelName "/"])
        record = struct("path", chart.Path, "name", chart.Name, "script", chart.Script);
        records(end + 1, 1) = record; %#ok<AGROW>
    end
end
end

function summary = compose_summary(inventory)
lines = [
    "Source: " + string(inventory.sourceModel)
    "Model: " + string(inventory.modelName)
    "Solver: " + string(inventory.configuration.solver)
    "Fixed step: " + string(inventory.configuration.fixedStep)
    "Stop time: " + string(inventory.configuration.stopTime)
    "Block count: " + string(inventory.blockCount)
    ""
    "Top-level connections:"
    ];
for index = 1:numel(inventory.topLevelConnections)
    item = inventory.topLevelConnections(index);
    lines(end + 1) = "  " + string(item.source) + ":" + string(item.sourcePort) + ...
        " -> " + string(item.destination) + ":" + string(item.destinationPort); %#ok<AGROW>
end
lines(end + 1) = "";
lines(end + 1) = "MATLAB Function charts:";
for index = 1:numel(inventory.stateflowCharts)
    lines(end + 1) = "  " + string(inventory.stateflowCharts(index).path) + ...
        "/" + string(inventory.stateflowCharts(index).name); %#ok<AGROW>
end
summary = strjoin(lines, newline);
end

function write_text(path, contents)
file = fopen(path, "w", "n", "UTF-8");
assert(file >= 0, "Unable to open %s", path);
cleanup = onCleanup(@() fclose(file)); %#ok<NASGU>
fprintf(file, "%s", contents);
end
