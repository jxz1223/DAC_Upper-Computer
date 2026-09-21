classdef TestReadSensorWaveformCsv < matlab.unittest.TestCase
    % CSV fixtures match the actual SensorWaveformViewer export headers.
    methods (TestClassSetup)
        function addSource(testCase)
            source = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(source));
        end
    end

    methods (Test)
        function readsBomAndPreservesIdentityAndPrecision(testCase)
            filename = testCase.writeCsv([ ...
                "node_id,channel,timestamp,elapsed_s,seq,dac,lockin,adc,display_field,display_value"
                "0x0010,2,1002,2,1,4000,12.34,60000,adc,60000"
                "0x000A,1,1000,0,1,4000,10.01,50000,adc,50000"
                "0x0002,1,1001,1,1,4000,11.02,50000,adc,50000"], true);

            actual = read_sensor_waveform_csv(filename);

            testCase.verifyEqual(actual.Format, "multi-node");
            testCase.verifyEqual(actual.Sensors.NodeId, ["0x0002";"0x000A";"0x0010"]);
            testCase.verifyEqual(actual.Sensors.Channel, [1;1;2], 'AbsTol', 0);
            testCase.verifyEqual(actual.Time_s, [0;1;2], 'AbsTol', 1e-12);
            testCase.verifyEqual(actual.Values, [10.01;11.02;12.34], 'AbsTol', 1e-12);
            testCase.verifyEqual(actual.SensorIndex, [2;1;3], 'AbsTol', 0);
            testCase.verifyEmpty(actual.Warnings);
        end

        function mergesSameNodeAcrossChannels(testCase)
            filename = testCase.writeCsv([ ...
                "node_id,channel,elapsed_s,lockin"
                "0x000a,2,0,1"
                "10,1,1,2"
                "0x000B,,2,3"], false);

            actual = read_sensor_waveform_csv(filename);

            testCase.verifyEqual(actual.Sensors.NodeId, ["0x000A";"0x000B"]);
            testCase.verifyEqual(actual.Sensors.Channel(1), 1, 'AbsTol', 0);
            testCase.verifyTrue(isnan(actual.Sensors.Channel(2)));
            testCase.verifyEqual(actual.SensorIndex, [1;1;2], 'AbsTol', 0);
            testCase.verifyNumElements(actual.Warnings, 2);
        end

        function readsSingleNodeAndFiltersOnlyInvalidSamples(testCase)
            filename = testCase.writeCsv([ ...
                "timestamp,elapsed_s,seq,dac,lockin,adc"
                "100,0,1,,1.23,broken"
                "101,1,2,4000,bad,9"
                "102,2,3,4000,Inf,9"
                "103,3,4,4000,4.56,9"], false);

            actual = read_sensor_waveform_csv(filename);

            testCase.verifyEqual(actual.Format, "single-node");
            testCase.verifyEqual(actual.Sensors.NodeId, "single");
            testCase.verifyEqual(actual.Time_s, [0;3], 'AbsTol', 1e-12);
            testCase.verifyEqual(actual.Values, [1.23;4.56], 'AbsTol', 1e-12);
            testCase.verifyNumElements(actual.Warnings, 1);
        end

        function recoversRelativeTimeUsingSharedTimestampOffset(testCase)
            filename = testCase.writeCsv([ ...
                "timestamp,elapsed_s,lockin"
                "1000,10,1"
                "1001,broken,2"
                "1002,12,3"], false);

            actual = read_sensor_waveform_csv(filename);

            testCase.verifyEqual(actual.Time_s, [10;11;12], 'AbsTol', 1e-12);
            testCase.verifyNumElements(actual.Warnings, 1);
        end

        function fallsBackToEpochTimestamp(testCase)
            filename = testCase.writeCsv([ ...
                "timestamp,lockin"
                "1700000002.25,3"
                "1700000000.25,1"
                "1700000001.25,2"], false);

            actual = read_sensor_waveform_csv(filename);

            testCase.verifyEqual(actual.Time_s, [0;1;2], 'AbsTol', 1e-12);
            testCase.verifyEqual(actual.Values, [1;2;3], 'AbsTol', 1e-12);
        end

        function retainsSensorWithEntirelyInvalidSamples(testCase)
            filename = testCase.writeCsv([ ...
                "node_id,channel,elapsed_s,lockin"
                "0x0001,1,0,10"
                "0x0002,2,1,bad"
                "wrong,3,2,20"], false);

            actual = read_sensor_waveform_csv(filename);

            testCase.verifyEqual(actual.Sensors.NodeId, ["0x0001";"0x0002"]);
            testCase.verifyEqual(actual.Values, 10, 'AbsTol', 1e-12);
            testCase.verifyEqual(actual.SensorIndex, 1, 'AbsTol', 0);
            testCase.verifyNumElements(actual.Warnings, 2);
        end

        function rejectsRawAcCsv(testCase)
            filename = testCase.writeCsv(["sample_index,time_s,value";"0,0,100"], false);

            testCase.verifyError(@() read_sensor_waveform_csv(filename), ...
                'SensorWaveform:MissingLockin');
        end

        function rejectsDacScanCsv(testCase)
            filename = testCase.writeCsv(["node_id,index,dac,lockin";"0x0001,0,1000,12.34"], false);

            testCase.verifyError(@() read_sensor_waveform_csv(filename), ...
                'SensorWaveform:MissingTime');
        end

        function rejectsNoUsableSamples(testCase)
            filename = testCase.writeCsv(["elapsed_s,lockin";"NaN,1";"1,NaN"], false);

            testCase.verifyError(@() read_sensor_waveform_csv(filename), ...
                'SensorWaveform:NoValidData');
        end
    end

    methods (Access = private)
        function filename = writeCsv(testCase, rows, withBom)
            filename = string(tempname) + ".csv";
            testCase.addTeardown(@() delete(filename));
            [fid,message] = fopen(filename, 'w', 'n', 'UTF-8');
            testCase.assertGreaterThanOrEqual(fid, 0, message);
            cleanup = onCleanup(@() fclose(fid));
            if withBom
                fwrite(fid, uint8([239 187 191]), 'uint8');
            end
            fprintf(fid, '%s\n', rows);
            clear cleanup
        end
    end
end
