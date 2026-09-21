classdef TestSensorVoltageApp < matlab.uitest.TestCase
    % Public-interface checks for the complete CSV -> UI -> coefficient path.
    properties
        CsvFile
        OutputFile
        App
        Ranges
    end

    methods (TestMethodSetup)
        function createApp(testCase)
            source = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(source));
            testCase.CsvFile = string(tempname) + ".csv";
            testCase.OutputFile = string(tempname) + ".csv";
            testCase.addTeardown(@() TestSensorVoltageApp.deleteOne(testCase.CsvFile));
            testCase.addTeardown(@() TestSensorVoltageApp.deleteOne(testCase.OutputFile));
            [~, testCase.Ranges] = create_demo_sensor_csv(testCase.CsvFile);
            testCase.App = SensorVoltageApp(testCase.CsvFile);
            testCase.addTeardown(@() delete(testCase.App));
            drawnow;
        end
    end

    methods (Test)
        function mouseSelectionFitsFiveVoltageStages(testCase)
            fig = findall(groot, 'Tag', 'SensorVoltageMain');
            ax = findall(fig, 'Tag', 'WaveAxes');
            button = findall(fig, 'Tag', 'PickRange');
            testCase.pickRange(button, ax, [1.5 10.5]);
            testCase.pickRange(button, ax, [13.5 22.5]);
            testCase.pickRange(button, ax, [25.5 34.5]);
            testCase.pickRange(button, ax, [37.5 46.5]);
            testCase.pickRange(button, ax, [49.5 58.5]);
            tolerance = 2 * diff(ax.XLim) / ax.InnerPosition(3);
            expected = [(0:12:48)' + 1.5, (0:12:48)' + 10.5];
            testCase.verifyEqual(testCase.App.Ranges, expected, 'AbsTol', tolerance);
            testCase.press(findall(fig, 'Tag', 'FitVoltage'));
            drawnow;
            testCase.verifyEqual(testCase.App.Results.k, [0.5; 1.2; -0.1], 'AbsTol', 1e-12);
        end
        function allSensorsShareOneAxes(testCase)
            fig = findall(groot, 'Tag', 'SensorVoltageMain');
            ax = findall(fig, 'Tag', 'WaveAxes');
            testCase.verifyNumElements(ax, 1);
            testCase.verifyNumElements(findall(ax, 'Type', 'line'), 3);
            testCase.verifyEqual(height(testCase.App.Data.Sensors), 3);
        end

        function fitButtonOpensSeparateWindow(testCase)
            testCase.App.setRanges(testCase.Ranges);
            fig = findall(groot, 'Tag', 'SensorVoltageMain');
            button = findall(fig, 'Tag', 'FitVoltage');
            button.ButtonPushedFcn(button, []);
            drawnow;
            fitFigure = findall(groot, 'Tag', 'SensorVoltageFit');
            testCase.verifyNumElements(fitFigure, 1);
            testCase.verifyNotEqual(fig, fitFigure);
            testCase.verifyNumElements(findall(fitFigure, 'Type', 'axes'), 3);
            testCase.verifyEqual(testCase.App.Results.k, [0.5; 1.2; -0.1], 'AbsTol', 1e-12);
            testCase.verifyEqual(testCase.App.Results.b, [10; 20; 130], 'AbsTol', 1e-10);
            testCase.verifyEqual(testCase.App.Results.R2, ones(3, 1), 'AbsTol', 1e-12);
            previewFolder = fileparts(fileparts(mfilename('fullpath')));
            exportapp(fig, fullfile(previewFolder, 'preview_waveforms.png'));
            exportapp(fitFigure, fullfile(previewFolder, 'preview_fits.png'));
        end

        function savedCoefficientsPreserveOrder(testCase)
            testCase.App.setRanges(testCase.Ranges);
            testCase.App.runFit();
            testCase.App.saveCoefficients(testCase.OutputFile);
            options = detectImportOptions(testCase.OutputFile, 'TextType', 'string');
            options = setvartype(options, 'NodeId', 'string');
            saved = readtable(testCase.OutputFile, options);
            testCase.verifyEqual(saved.Order, (1:3)');
            testCase.verifyEqual(saved.NodeId, ["0x0011"; "0x0022"; "0x0033"]);
            testCase.verifyEqual(saved.k, [0.5; 1.2; -0.1], 'AbsTol', 1e-12);
            testCase.verifyEqual(saved.b, [10; 20; 130], 'AbsTol', 1e-10);
        end

        function changingRangesInvalidatesOldFit(testCase)
            testCase.App.setRanges(testCase.Ranges);
            testCase.App.runFit();
            testCase.App.setRanges(testCase.Ranges + 0.1);
            testCase.verifyEmpty(testCase.App.Results);
            testCase.verifyEmpty(findall(groot, 'Tag', 'SensorVoltageFit'));
            testCase.verifyError(@() testCase.App.saveCoefficients(testCase.OutputFile), ...
                'SensorVoltage:NoFit');
        end

        function escapeCancelsPickingAfterZoom(testCase)
            fig = findall(groot, 'Tag', 'SensorVoltageMain');
            zoom(fig, 'on');
            button = findall(fig, 'Tag', 'PickRange');
            button.ButtonPushedFcn(button, []);
            testCase.verifyEqual(button.Text, '取消选择 (Esc)');
            testCase.verifyEqual(fig.Pointer, 'crosshair');
            fig.WindowKeyPressFcn(fig, struct('Key', 'escape'));
            testCase.verifyEqual(button.Text, '在曲线上选区间');
            testCase.verifyEqual(fig.Pointer, 'arrow');
            testCase.verifyTrue(all(isnan(testCase.App.Ranges), 'all'));
        end

        function relativeSourcePathIsFixedAtLoadTime(testCase)
            [folder, name, extension] = fileparts(testCase.CsvFile);
            testCase.applyFixture(matlab.unittest.fixtures.CurrentFolderFixture(folder));
            data = read_sensor_waveform_csv(name + extension);
            testCase.verifyEqual(data.SourceFile, testCase.CsvFile);
        end
        function cannotOverwriteSourceCsv(testCase)
            original = fileread(testCase.CsvFile);
            testCase.App.setRanges(testCase.Ranges);
            testCase.App.runFit();
            testCase.verifyError(@() testCase.App.saveCoefficients(testCase.CsvFile), ...
                'SensorVoltage:SourceOverwrite');
            testCase.verifyEqual(fileread(testCase.CsvFile), original);
        end
    end

    methods (Access = private)
        function pickRange(testCase, button, ax, interval)
            testCase.press(button);
            drawnow;
            y = mean(ax.YLim);
            testCase.press(ax, [interval(1), y], 'SelectionType', 'normal');
            drawnow;
            testCase.press(ax, [interval(2), y], 'SelectionType', 'normal');
            drawnow;
        end
    end
    methods (Static, Access = private)
        function deleteOne(file)
            if isfile(file)
                delete(file);
            end
        end
    end
end
