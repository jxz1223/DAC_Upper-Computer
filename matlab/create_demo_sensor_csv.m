function [filePath, ranges] = create_demo_sensor_csv(filePath)
%CREATE_DEMO_SENSOR_CSV 生成与上位机格式一致的三传感器模拟数据。
%   [file, ranges] = create_demo_sensor_csv;
%   app = SensorVoltageApp(file); app.setRanges(ranges); app.runFit();
%   这是人工演示数据，不是传感器实测结果。
%   真实系数依次为 k=[0.50,1.20,-0.10], b=[10,20,130]。
    if nargin < 1
        filePath = fullfile(tempdir, 'sensor_voltage_demo.csv');
    end
    voltages = [100; 200; 300; 500; 1000];
    slopes = [0.50, 1.20, -0.10];
    offsets = [10, 20, 130];
    ranges = [(0:12:48)' + 2, (0:12:48)' + 10.5];
    node_id = strings(0, 1);
    channel = zeros(0, 1);
    elapsed_s = zeros(0, 1);
    lockin = zeros(0, 1);
    for stage = 1:5
        for sample = 0:11
            for sensor = 1:3
                time = (stage - 1) * 12 + sample + (sensor - 1) * 0.05;
                value = slopes(sensor) * voltages(stage) + offsets(sensor);
                if sample < 2 && stage > 1
                    previous = slopes(sensor) * voltages(stage - 1) + offsets(sensor);
                    value = previous + (value - previous) * (sample + 1) / 3;
                elseif sample >= 2 && sample <= 10
                    value = value + 0.2 * sin(2 * pi * (sample - 2) / 9);
                end
                node_id(end + 1, 1) = sprintf('0x%04X', sensor * 17); %#ok<AGROW>
                channel(end + 1, 1) = sensor; %#ok<AGROW>
                elapsed_s(end + 1, 1) = time; %#ok<AGROW>
                lockin(end + 1, 1) = round(value, 2); %#ok<AGROW>
            end
        end
    end
    timestamp = 1800000000 + elapsed_s;
    seq = (1:numel(lockin))';
    dac = repmat(2048, size(seq));
    adc = repmat(1234, size(seq));
    display_field = repmat("lockin", size(seq));
    display_value = lockin;
    data = table(node_id, channel, timestamp, elapsed_s, seq, dac, ...
        lockin, adc, display_field, display_value);
    writetable(data, filePath, 'Encoding', 'UTF-8');
    filePath = string(filePath);
end
