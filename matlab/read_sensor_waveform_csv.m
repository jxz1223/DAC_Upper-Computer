function data = read_sensor_waveform_csv(filename)
%READ_SENSOR_WAVEFORM_CSV Read the viewer's lock-in waveform CSV exports.
% DATA = READ_SENSOR_WAVEFORM_CSV(FILE) supports multi-node V2 exports and
% single-node DC exports. Lock-in values already contain the viewer's scale;
% no conversion using DAC, ADC, display_value, or an assumed voltage occurs.
%
% Output fields:
%   SourceFile  - input filename (string)
%   Format      - "multi-node" or "single-node"
%   Sensors     - table with NodeId (string), Channel (double), Label (string)
%   Time_s      - elapsed seconds, sorted; one row per usable sample
%   Values      - lock-in output in the CSV's original units
%   SensorIndex - index into Sensors for each sample
%   Warnings    - string column describing recovered/discarded data
%
% Sensor identity is NodeId, never channel: a channel can be reused by a
% different sensor. Sensors are ordered by valid channel, then numeric NodeId.
% Requires MATLAB R2020b or newer; no additional toolbox is required.

arguments
    filename (1,1) string
end

if ~isfile(filename)
    error('SensorWaveform:FileNotFound', 'CSV 文件不存在：%s', filename);
end

[~, fileInfo] = fileattrib(filename);
filename = string(fileInfo.Name);

try
    opts = detectImportOptions(filename, 'Delimiter', ',', ...
        'VariableNamingRule', 'preserve', 'Encoding', 'UTF-8');
    % Import as text so one malformed entry never changes an entire column's
    % interpretation. Convert only the fields relevant to this measurement.
    opts = setvartype(opts, opts.VariableNames, 'string');
    opts.VariableNamesLine = 1;
    opts.DataLines = [2 Inf];
    opts.ExtraColumnsRule = 'error';
    raw = readtable(filename, opts);
catch cause
    exception = MException('SensorWaveform:InvalidCsv', ...
        '无法读取 CSV：%s', cause.message);
    throwAsCaller(addCause(exception, cause));
end

names = lower(strip(erase(string(raw.Properties.VariableNames), char(65279))));
if ~any(names == "lockin")
    error('SensorWaveform:MissingLockin', ...
        ['CSV 缺少 lockin 列。请选择“导出多节点 CSV”或直流“保存数据”的文件。' ...
         '一对一交流原始 ADC 数据不能作为锁相输出。']);
end
if ~any(names == "elapsed_s") && ~any(names == "timestamp")
    error('SensorWaveform:MissingTime', ...
        'CSV 缺少 elapsed_s/timestamp 时间列；DAC 扫描点文件不属于实时波形文件。');
end
if isempty(raw)
    error('SensorWaveform:NoValidData', 'CSV 中没有数据行。');
end

nRows = height(raw);
warnings = strings(0,1);
values = numericColumn(raw, names, "lockin", nRows);
elapsed = numericColumn(raw, names, "elapsed_s", nRows);
timestamps = numericColumn(raw, names, "timestamp", nRows);
time = elapsed;
validElapsed = isfinite(elapsed);
validTimestamp = isfinite(timestamps);

if ~any(validElapsed)
    if any(validTimestamp)
        time = timestamps - min(timestamps(validTimestamp));
        warnings(end+1,1) = "elapsed_s 不可用，已使用 timestamp 从首个有效时间戳计算相对秒数。";
    end
else
    anchors = validElapsed & validTimestamp;
    recover = ~validElapsed & validTimestamp;
    if any(anchors) && any(recover)
        % Export writes epoch seconds and relative seconds to six decimal
        % places, so their common offset aligns recovered rows precisely.
        offset = median(timestamps(anchors) - elapsed(anchors));
        time(recover) = timestamps(recover) - offset;
        warnings(end+1,1) = string(sprintf( ...
            '已用 timestamp 恢复 %d 行缺失或无效的 elapsed_s。', nnz(recover)));
    end
end

multiNode = any(names == "node_id");
if multiNode
    rawIds = strip(string(raw{:,find(names == "node_id",1)}));
    rawIds(ismissing(rawIds)) = "";
    [idStrings,~,idLookup] = unique(upper(rawIds));
    numericIds = nan(size(idStrings));
    for k = 1:numel(idStrings)
        token = char(idStrings(k));
        if ~isempty(regexp(token, '^0X[0-9A-F]{1,4}$', 'once'))
            numericIds(k) = hex2dec(token(3:end));
        else
            numericIds(k) = str2double(token);
        end
    end
    nodeNumbers = numericIds(idLookup);
    validId = isfinite(nodeNumbers) & nodeNumbers >= 0 & ...
        nodeNumbers <= 65535 & nodeNumbers == fix(nodeNumbers);
    channels = numericColumn(raw, names, "channel", nRows);
    validChannel = isfinite(channels) & channels >= 1 & channels == fix(channels);
    channels(~validChannel) = NaN;
    if any(~validChannel)
        warnings(end+1,1) = string(sprintf( ...
            '%d 行通道编号不可用；仍按 NodeId 识别传感器，无有效通道的传感器排在末尾。', ...
            nnz(~validChannel)));
    end
    format = "multi-node";
else
    nodeNumbers = ones(nRows,1);
    validId = true(nRows,1);
    channels = ones(nRows,1);
    format = "single-node";
end

valid = validId & isfinite(time) & isfinite(values);
if any(~valid)
    warnings(end+1,1) = string(sprintf( ...
        ['已忽略 %d/%d 行无效样点（NodeId 无效 %d 行、时间无效 %d 行、' ...
         'lockin 无效 %d 行；原因可能重叠）。'], ...
        nnz(~valid), nRows, nnz(~validId), nnz(~isfinite(time)), nnz(~isfinite(values))));
end
if ~any(valid)
    error('SensorWaveform:NoValidData', ...
        'CSV 没有同时包含有效 NodeId、时间和 lockin 的样点。');
end

% Keep a sensor in the catalog even if all of its samples were discarded;
% the fitting layer can then report incomplete calibration for that sensor.
uniqueNodes = unique(nodeNumbers(validId));
sensorChannels = nan(numel(uniqueNodes),1);
labels = strings(numel(uniqueNodes),1);
nodeIds = strings(numel(uniqueNodes),1);
for k = 1:numel(uniqueNodes)
    rows = validId & nodeNumbers == uniqueNodes(k);
    possibleChannels = unique(channels(rows & isfinite(channels)));
    if ~isempty(possibleChannels)
        sensorChannels(k) = possibleChannels(1);
    end
    if multiNode
        nodeIds(k) = string(sprintf('0x%04X', uniqueNodes(k)));
    else
        nodeIds(k) = "single";
    end
    if numel(possibleChannels) > 1
        warnings(end+1,1) = "NodeId " + nodeIds(k) + ...
            " 对应多个 CH 编号，按同一传感器合并；排序使用最小有效 CH。";
    end
    if isfinite(sensorChannels(k))
        labels(k) = string(sprintf('CH%d (%s)', sensorChannels(k), nodeIds(k)));
    else
        labels(k) = "NodeId " + nodeIds(k);
    end
    if ~any(rows & valid)
        warnings(end+1,1) = "传感器 " + labels(k) + " 没有有效样点，已保留其身份以提示标定缺失。";
    end
end

channelSort = sensorChannels;
channelSort(~isfinite(channelSort)) = Inf;
[~,sensorOrder] = sortrows([channelSort, uniqueNodes], [1 2]);
sensors = table(nodeIds(sensorOrder), sensorChannels(sensorOrder), labels(sensorOrder), ...
    'VariableNames', {'NodeId','Channel','Label'});
[~,sensorIndex] = ismember(nodeNumbers(valid), uniqueNodes(sensorOrder));
time = time(valid);
values = values(valid);
[time,sampleOrder] = sort(time, 'ascend');

data = struct('SourceFile', filename, 'Format', format, 'Sensors', sensors, ...
    'Time_s', time(:), 'Values', values(sampleOrder), ...
    'SensorIndex', sensorIndex(sampleOrder), 'Warnings', warnings);
end

function values = numericColumn(raw, names, requestedName, nRows)
index = find(names == requestedName, 1);
if isempty(index)
    values = nan(nRows,1);
else
    values = str2double(string(raw{:,index}));
    values = values(:);
end
end
