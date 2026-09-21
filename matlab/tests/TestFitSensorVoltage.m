classdef TestFitSensorVoltage < matlab.unittest.TestCase
    % Verify calibration, interval boundaries and incomplete recordings.

    properties (TestParameter)
        invalidRanges = struct( ...
            'tooFewRows', [0 1; 1 2; 2 3; 3 4], ...
            'zeroWidth', [0 1; 1 2; 2 2; 3 4; 4 5], ...
            'reverseWidth', [0 1; 2 1; 2 3; 3 4; 4 5], ...
            'notFinite', [0 1; 1 2; 2 NaN; 3 4; 4 5], ...
            'overlap', [0 1.1; 1 2; 2 3; 3 4; 4 5], ...
            'outOfOrder', [1 2; 0 1; 2 3; 3 4; 4 5]);
    end

    methods (TestClassSetup)
        function addSourceToPath(testCase)
            sourceFolder = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(sourceFolder));
        end
    end

    methods (Test)
        function fitsSensorsAndPreservesSensorOrder(testCase)
            data = TestFitSensorVoltage.twoSensors();
            ranges = [(0:4).', (1:5).'];

            [results, points] = fit_sensor_voltage(data, ranges);

            testCase.verifyEqual(results.Properties.VariableNames, ...
                {'Order', 'NodeId', 'Channel', 'k', 'b', 'R2', 'r', 'RMSE', 'Status'});
            testCase.verifyEqual(results.Order, [1; 2]);
            testCase.verifyEqual(results.NodeId, ["200"; "001"]);
            testCase.verifyEqual(results.Channel, [9; 1]);
            testCase.verifyEqual(results.k, [2; -0.5], 'AbsTol', 1e-12);
            testCase.verifyEqual(results.b, [3; 7], 'AbsTol', 1e-10);
            testCase.verifyEqual(results.R2, [1; 1], 'AbsTol', 1e-12);
            testCase.verifyEqual(results.r, [1; -1], 'AbsTol', 1e-12);
            testCase.verifyEqual(results.RMSE, [0; 0], 'AbsTol', 1e-10);
            testCase.verifyEqual(results.Status, ["成功"; "成功"]);
            testCase.verifyEqual(points.Order, [ones(5, 1); 2 * ones(5, 1)]);
            testCase.verifyEqual(points.Voltage_V, repmat([100; 200; 300; 500; 1000], 2, 1));
        end

        function usesEqualWeightForEachVoltage(testCase)
            data = TestFitSensorVoltage.oneSensor([1; 4; 2; 8; 5], [1; 2; 3; 4; 5]);
            ranges = [(0:4).', (1:5).'];

            [results, points] = fit_sensor_voltage(data, ranges);

            testCase.verifyEqual(points.Count, [1; 2; 3; 4; 5]);
            testCase.verifyEqual(points.Mean, [1; 4; 2; 8; 5], 'AbsTol', 1e-12);
            testCase.verifyEqual(results.k, 2100 / 508000, 'AbsTol', 1e-12);
            testCase.verifyEqual(results.b, 4 - 420 * 2100 / 508000, 'AbsTol', 1e-12);
            testCase.verifyEqual(results.R2, (2100^2 / 508000) / 30, 'AbsTol', 1e-12);
            testCase.verifyEqual(results.r, sqrt((2100^2 / 508000) / 30), 'AbsTol', 1e-12);
            testCase.verifyEqual(results.RMSE, sqrt((30 - 2100^2 / 508000) / 5), 'AbsTol', 1e-12);
        end

        function excludesNonfiniteSamplesAndReportsSampleStd(testCase)
            data = TestFitSensorVoltage.oneSensor([10; 20; 30; 40; 50], [5; 1; 1; 1; 1]);
            data.Values(1:5) = [8; 12; NaN; Inf; -Inf];
            ranges = [(0:4).', (1:5).'];

            [results, points] = fit_sensor_voltage(data, ranges);

            testCase.verifyEqual(points.Count, [2; 1; 1; 1; 1]);
            testCase.verifyEqual(points.Mean, [10; 20; 30; 40; 50], 'AbsTol', 1e-12);
            testCase.verifyEqual(points.Std, [sqrt(8); 0; 0; 0; 0], 'AbsTol', 1e-12);
            testCase.verifyTrue(isfinite(results.k));
        end

        function sharedBoundariesAreCountedOnce(testCase)
            data = TestFitSensorVoltage.oneSensor([100; 200; 300; 500; 1000], ones(5, 1));
            data.Time_s = (0:5).';
            data.Values = [100; 200; 300; 500; 1000; 1000];
            data.SensorIndex = ones(6, 1);
            ranges = [(0:4).', (1:5).'];

            [results, points] = fit_sensor_voltage(data, ranges);

            testCase.verifyEqual(points.Count, [1; 1; 1; 1; 2]);
            testCase.verifyEqual(points.Mean, [100; 200; 300; 500; 1000], 'AbsTol', 1e-12);
            testCase.verifyEqual(results.k, 1, 'AbsTol', 1e-12);
            testCase.verifyEqual(results.b, 0, 'AbsTol', 1e-12);
        end

        function missingVoltageDoesNotDropOrBlockOtherSensors(testCase)
            data = TestFitSensorVoltage.twoSensors();
            data.Values(8) = NaN;
            ranges = [(0:4).', (1:5).'];

            [results, points] = fit_sensor_voltage(data, ranges);

            testCase.verifyEqual(height(results), 2);
            testCase.verifyEqual(results.k(1), 2, 'AbsTol', 1e-12);
            testCase.verifyTrue(all(isnan(results{2, {'k', 'b', 'R2', 'r', 'RMSE'}})));
            testCase.verifyTrue(contains(results.Status(2), "300 V"));
            testCase.verifyEqual(points.Count(8), 0);
            testCase.verifyTrue(isnan(points.Mean(8)));
        end

        function constantOutputHasUndefinedCorrelation(testCase)
            data = TestFitSensorVoltage.oneSensor(17 * ones(5, 1), ones(5, 1));
            ranges = [(0:4).', (1:5).'];

            results = fit_sensor_voltage(data, ranges);

            testCase.verifyEqual(results.k, 0, 'AbsTol', 1e-12);
            testCase.verifyEqual(results.b, 17, 'AbsTol', 1e-12);
            testCase.verifyEqual(results.RMSE, 0, 'AbsTol', 1e-12);
            testCase.verifyTrue(isnan(results.R2));
            testCase.verifyTrue(isnan(results.r));
            testCase.verifyTrue(contains(results.Status, "均值恒定"));
        end

        function allMissingSamplesPreserveResultsRows(testCase)
            data = TestFitSensorVoltage.twoSensors();
            data.Values(:) = NaN;
            ranges = [(0:4).', (1:5).'];

            [results, points] = fit_sensor_voltage(data, ranges);

            testCase.verifyEqual(height(results), 2);
            testCase.verifyTrue(all(isnan(results.k)));
            testCase.verifyEqual(points.Count, zeros(10, 1));
            testCase.verifyTrue(all(contains(results.Status, "缺少有效数据")));
        end

        function rejectsInvalidRanges(testCase, invalidRanges)
            data = TestFitSensorVoltage.twoSensors();

            testCase.verifyError(@() fit_sensor_voltage(data, invalidRanges), ...
                'SensorVoltage:InvalidRanges');
        end
    end

    methods (Static, Access = private)
        function data = twoSensors()
            x = [100; 200; 300; 500; 1000];
            data.Sensors = table(["200"; "001"], [9; 1], ["first"; "second"], ...
                'VariableNames', {'NodeId', 'Channel', 'Label'});
            data.Time_s = repmat((0:4).' + 0.25, 2, 1);
            data.Values = [2 * x + 3; -0.5 * x + 7];
            data.SensorIndex = [ones(5, 1); 2 * ones(5, 1)];
        end

        function data = oneSensor(means, counts)
            data.Sensors = table("01", 1, "sensor 01", ...
                'VariableNames', {'NodeId', 'Channel', 'Label'});
            data.Time_s = zeros(sum(counts), 1);
            offset = 0;
            for stage = 1:5
                rows = offset + (1:counts(stage));
                data.Time_s(rows) = stage - 1 + (1:counts(stage)).' / (counts(stage) + 1);
                offset = offset + counts(stage);
            end
            data.Values = repelem(means, counts);
            data.SensorIndex = ones(sum(counts), 1);
        end
    end
end
