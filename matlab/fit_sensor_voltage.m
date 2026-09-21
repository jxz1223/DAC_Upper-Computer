function [results, points] = fit_sensor_voltage(data, ranges)
%FIT_SENSOR_VOLTAGE Fit each sensor's five stable means to y = k*x + b.
%   [RESULTS, POINTS] = FIT_SENSOR_VOLTAGE(DATA, RANGES) uses the five rows
%   of RANGES, in seconds, for 100, 200, 300, 500 and 1000 V respectively.
%   DATA is returned by READ_SENSOR_WAVEFORM_CSV. Each voltage contributes one mean
%   with equal weight, regardless of the number of samples in its interval.
%
%   Intervals are [start,end), except the final interval is [start,end].
%   RESULTS follows DATA.Sensors order, including sensors with missing data.
%   R2 is the coefficient of determination; r is Pearson's correlation;
%   RMSE is sqrt(mean(residual.^2)) over the five stable means.
%   POINTS.Std is the sample standard deviation within each stable interval.
%   No additional toolbox is required.

    voltages = [100; 200; 300; 500; 1000];
    stageCount = numel(voltages);
    validateRanges(ranges, stageCount);
    validateData(data);

    sensorCount = height(data.Sensors);
    sensorOrder = (1:sensorCount).';
    nodeId = string(data.Sensors.NodeId);
    channel = double(data.Sensors.Channel);
    unknown = nan(sensorCount, 1);
    results = table(sensorOrder, nodeId, channel, unknown, unknown, ...
        unknown, unknown, unknown, strings(sensorCount, 1), ...
        'VariableNames', {'Order', 'NodeId', 'Channel', 'k', 'b', ...
        'R2', 'r', 'RMSE', 'Status'});

    % Specify both dimensions so a single sensor still produces a column.
    pointOrder = repelem(sensorOrder, stageCount, 1);
    pointNodeId = repelem(nodeId, stageCount, 1);
    pointCount = sensorCount * stageCount;
    points = table(pointOrder, pointNodeId, ...
        repmat(voltages, sensorCount, 1), repmat(ranges(:, 1), sensorCount, 1), ...
        repmat(ranges(:, 2), sensorCount, 1), zeros(pointCount, 1), ...
        nan(pointCount, 1), nan(pointCount, 1), ...
        'VariableNames', {'Order', 'NodeId', 'Voltage_V', 'Start_s', ...
        'End_s', 'Count', 'Mean', 'Std'});

    time = double(data.Time_s(:));
    values = double(data.Values(:));
    sensorIndex = double(data.SensorIndex(:));
    stage = zeros(size(time));
    for index = 1:stageCount
        if index == stageCount
            inRange = time >= ranges(index, 1) & time <= ranges(index, 2);
        else
            inRange = time >= ranges(index, 1) & time < ranges(index, 2);
        end
        stage(inRange) = index;
    end

    valid = stage > 0 & isfinite(time) & isfinite(values);
    if any(valid)
        samples = table(sensorIndex(valid), stage(valid), values(valid), ...
            'VariableNames', {'SensorIndex', 'Stage', 'Value'});
        stats = groupsummary(samples, ["SensorIndex", "Stage"], ...
            ["mean", "std"], "Value");
        destination = (stats.SensorIndex - 1) * stageCount + stats.Stage;
        points.Count(destination) = stats.GroupCount;
        points.Mean(destination) = stats.mean_Value;
        points.Std(destination) = stats.std_Value;
    end

    xMean = mean(voltages);
    xCentered = voltages - xMean;
    sxx = sum(xCentered .^ 2);
    for index = 1:sensorCount
        rows = (index - 1) * stageCount + (1:stageCount);
        y = points.Mean(rows);
        missing = points.Count(rows) == 0;
        if any(missing)
            missingVoltages = join(string(voltages(missing)) + " V", "、");
            results.Status(index) = "缺少有效数据：" + missingVoltages;
            continue
        end
        if any(~isfinite(y))
            results.Status(index) = "区间均值非有限值，无法拟合";
            continue
        end
        if all(y == y(1))
            results.k(index) = 0;
            results.b(index) = y(1);
            results.RMSE(index) = 0;
            results.Status(index) = "均值恒定：R² 和 r 未定义";
            continue
        end

        yMean = mean(y);
        yCentered = y - yMean;
        syy = sum(yCentered .^ 2);
        sxy = sum(xCentered .* yCentered);
        slope = sxy / sxx;
        intercept = yMean - slope * xMean;
        residual = yCentered - slope * xCentered;
        sse = sum(residual .^ 2);
        results.k(index) = slope;
        results.b(index) = intercept;
        results.R2(index) = min(1, max(0, 1 - sse / syy));
        results.r(index) = min(1, max(-1, sxy / sqrt(sxx * syy)));
        results.RMSE(index) = sqrt(sse / stageCount);
        results.Status(index) = "成功";
    end
end

function validateRanges(ranges, stageCount)
    if ~isnumeric(ranges) || ~isreal(ranges) || ...
            ~isequal(size(ranges), [stageCount, 2]) || ...
            any(~isfinite(ranges(:)))
        error('SensorVoltage:InvalidRanges', ...
            '稳定区间必须是有限实数构成的 5×2 矩阵，依次对应 100、200、300、500、1000 V。');
    end
    if any(ranges(:, 1) >= ranges(:, 2))
        error('SensorVoltage:InvalidRanges', ...
            '每个稳定区间的起始时间必须小于结束时间。');
    end
    if any(ranges(2:end, 1) < ranges(1:end-1, 2))
        error('SensorVoltage:InvalidRanges', ...
            '五个稳定区间必须按施加电压的时间顺序排列，且不能重叠。');
    end
end

function validateData(data)
    required = {'Sensors', 'Time_s', 'Values', 'SensorIndex'};
    if ~isstruct(data) || ~isscalar(data) || ~all(isfield(data, required))
        error('SensorVoltage:InvalidData', '输入数据必须由 read_sensor_waveform_csv 读取。');
    end
    if ~istable(data.Sensors) || ...
            ~all(ismember({'NodeId', 'Channel'}, data.Sensors.Properties.VariableNames))
        error('SensorVoltage:InvalidData', 'Sensors 表必须包含 NodeId 和 Channel 列。');
    end
    numericFields = {'Time_s', 'Values', 'SensorIndex'};
    for index = 1:numel(numericFields)
        value = data.(numericFields{index});
        if ~isnumeric(value) || ~isreal(value) || (~isvector(value) && ~isempty(value))
            error('SensorVoltage:InvalidData', '时间、输出值和传感器索引必须是实数向量。');
        end
    end
    if numel(data.Time_s) ~= numel(data.Values) || ...
            numel(data.Time_s) ~= numel(data.SensorIndex)
        error('SensorVoltage:InvalidData', '时间、输出值和传感器索引的长度必须相同。');
    end
    index = data.SensorIndex(:);
    if any(~isfinite(index) | index < 1 | ...
            index > height(data.Sensors) | index ~= fix(index))
        error('SensorVoltage:InvalidData', '传感器索引必须对应 Sensors 表中的有效行号。');
    end
end
