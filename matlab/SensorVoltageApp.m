classdef SensorVoltageApp < handle
    %SENSORVOLTAGEAPP 多传感器锁相输出查看与五档电压线性标定。
    %   SensorVoltageApp 打开界面；点击“打开 CSV”读取上位机导出文件。
    %   app = SensorVoltageApp(csvFile) 直接打开指定文件。
    %   app.setRanges([t1 t2; t3 t4; t5 t6; t7 t8; t9 t10]);
    %   result = app.runFit(); app.saveCoefficients("coefficients.csv");
    %   时间单位为秒，与 CSV 的 elapsed_s 一致。

    properties (SetAccess = private)
        Data struct = struct([])
        Ranges double = nan(5, 2)
        Results table = table()
        Points table = table()
    end

    properties (Access = private)
        MainFigure matlab.ui.Figure
        FitFigure matlab.ui.Figure
        WaveAxes matlab.ui.control.UIAxes
        RangeTable matlab.ui.control.Table
        VoltageDropDown matlab.ui.control.DropDown
        PickButton matlab.ui.control.Button
        FitButton matlab.ui.control.Button
        FileLabel matlab.ui.control.Label
        StatusLabel matlab.ui.control.Label
        FitStatusLabel matlab.ui.control.Label
        Picking logical = false
        SelectionStart double = NaN
        RangeGraphics = gobjects(0)
        CursorGraphics = gobjects(0)
        Colors double = zeros(0, 3)
    end

    properties (Constant, Access = private)
        Voltages = [100; 200; 300; 500; 1000]
    end

    methods
        function app = SensorVoltageApp(csvFile)
            app.createComponents();
            if nargin > 0 && strlength(string(csvFile)) > 0
                try
                    app.loadCsv(csvFile);
                catch exception
                    delete(app);
                    rethrow(exception);
                end
            end
            if nargout == 0
                clear app
            end
        end

        function delete(app)
            if ~isempty(app.FitFigure) && isvalid(app.FitFigure)
                delete(app.FitFigure);
            end
            if ~isempty(app.MainFigure) && isvalid(app.MainFigure)
                delete(app.MainFigure);
            end
        end

        function loadCsv(app, csvFile)
            % 先完整解析，读取失败时保留原图及原标定结果。
            newData = read_sensor_waveform_csv(csvFile);
            app.stopPicking();
            app.invalidateFit();
            app.Data = newData;
            app.Ranges = nan(5, 2);
            app.Colors = lines(height(newData.Sensors));
            app.VoltageDropDown.Value = 1;
            app.RangeTable.Data = [app.Voltages, app.Ranges];
            app.FileLabel.Text = string(newData.SourceFile);
            app.FileLabel.Tooltip = string(newData.SourceFile);
            app.PickButton.Enable = 'on';
            app.FitButton.Enable = 'on';
            app.drawWaveforms();
            app.StatusLabel.Text = sprintf( ...
                '已读取 %d 个传感器、%d 个有效采样点。请依次选择五档电压的稳定区间。', ...
                height(newData.Sensors), numel(newData.Values));
            if ~isempty(newData.Warnings)
                app.StatusLabel.Text = app.StatusLabel.Text + " " + ...
                    strjoin(string(newData.Warnings), "；");
            end
        end

        function setRanges(app, ranges)
            % 可用于重复实验：时间边界与交互选择使用同一套计算。
            validateattributes(ranges, {'numeric'}, ...
                {'real', 'size', [5 2]}, mfilename, 'ranges');
            app.stopPicking();
            app.Ranges = double(ranges);
            app.RangeTable.Data = [app.Voltages, app.Ranges];
            app.invalidateFit();
            app.drawRanges();
        end

        function results = runFit(app)
            if isempty(app.Data)
                error('SensorVoltage:NoData', '请先打开上位机导出的 CSV 文件。');
            end
            app.stopPicking();
            [results, points] = fit_sensor_voltage(app.Data, app.Ranges);
            app.Results = results;
            app.Points = points;
            app.showFitWindow();
            app.StatusLabel.Text = '拟合已完成。请在独立的拟合窗口查看并保存系数。';
        end

        function saveCoefficients(app, filePath)
            if isempty(app.Results)
                error('SensorVoltage:NoFit', '请先完成拟合，再保存系数。');
            end
            if ~((ischar(filePath) && isrow(filePath)) || (isstring(filePath) && isscalar(filePath) && ~ismissing(filePath)))
                error('SensorVoltage:InvalidPath', '请提供一个有效的 CSV 文件路径。');
            end
            % 防止误选原始数据文件；不允许用标定表覆盖采集数据。
            destinationFile = java.io.File(char(filePath));
            destination = destinationFile.getCanonicalPath();
            sourceFile = java.io.File(char(app.Data.SourceFile));
            source = sourceFile.getCanonicalPath();
            if strcmpi(char(destination), char(source))
                error('SensorVoltage:SourceOverwrite', ...
                    '系数文件不能覆盖原始采集 CSV，请选择其他文件名。');
            end
            writetable(app.Results, filePath, 'Encoding', 'UTF-8');
        end
    end

    methods (Access = private)
        function createComponents(app)
            app.MainFigure = uifigure('Name', '传感器锁相输出与电压标定', ...
                'Position', [80 70 1180 820], 'Tag', 'SensorVoltageMain', ...
                'CloseRequestFcn', @(~, ~) delete(app), ...
                'WindowKeyPressFcn', @(~, event) app.keyPressed(event));
            root = uigridlayout(app.MainFigure, [5 1]);
            root.RowHeight = {42, 40, '1x', 240, 44};
            root.Padding = [14 12 14 12];
            top = uigridlayout(root, [1 3]);
            top.Layout.Row = 1;
            top.ColumnWidth = {130, '1x', 140};
            top.Padding = [0 0 0 0];
            uibutton(top, 'Text', '打开 CSV', 'Tag', 'OpenCsv', ...
                'ButtonPushedFcn', @(~, ~) app.chooseCsv());
            uilabel(top, 'Text', '多传感器锁相输出  /  五档电压标定', ...
                'FontSize', 18, 'FontWeight', 'bold');
            uibutton(top, 'Text', '显示完整曲线', ...
                'ButtonPushedFcn', @(~, ~) app.resetView());
            app.FileLabel = uilabel(root, 'Text', '请选择上位机“导出多节点 CSV”保存的文件。', ...
                'WordWrap', 'on', 'Interpreter', 'none');
            app.FileLabel.Layout.Row = 2;
            app.WaveAxes = uiaxes(root, 'Tag', 'WaveAxes', ...
                'ButtonDownFcn', @(~, ~) app.waveClicked());
            app.WaveAxes.Layout.Row = 3;
            title(app.WaveAxes, '各传感器锁相输出');
            xlabel(app.WaveAxes, '相对时间 / s');
            ylabel(app.WaveAxes, '锁相输出（CSV 原值）');
            grid(app.WaveAxes, 'on');

            panel = uipanel(root, 'Title', '选择稳定区间（可在表格中精确修改时间）');
            panel.Layout.Row = 4;
            body = uigridlayout(panel, [1 2]);
            body.ColumnWidth = {'1x', 380};
            app.RangeTable = uitable(body, 'Data', [app.Voltages, app.Ranges], ...
                'ColumnName', {'施加电压 / V', '开始 / s', '结束 / s'}, ...
                'ColumnEditable', [false true true], 'RowName', [], ...
                'ColumnWidth', {130, 'auto', 'auto'}, 'Tag', 'VoltageRanges', ...
                'CellEditCallback', @(~, event) app.rangeEdited(event));
            app.RangeTable.Layout.Column = 1;
            controls = uigridlayout(body, [4 2]);
            controls.Layout.Column = 2;
            controls.RowHeight = {'1x', 32, 32, 36};
            controls.ColumnWidth = {'1x', '1x'};
            controls.Padding = [0 0 0 0];
            hint = uilabel(controls, 'WordWrap', 'on', 'Text', ...
                ['先选电压，再点击“在曲线上选区间”，依次点击稳定段的起点、终点。' ...
                 '按 Esc 取消。五个区间必须按时间递增且不重叠；每档取区间均值。']);
            hint.Layout.Row = 1;
            hint.Layout.Column = [1 2];
            app.VoltageDropDown = uidropdown(controls, ...
                'Items', {'100 V', '200 V', '300 V', '500 V', '1000 V'}, ...
                'ItemsData', 1:5, 'Value', 1, 'Tag', 'VoltageChoice', ...
                'ValueChangedFcn', @(~, ~) app.stopPicking());
            app.VoltageDropDown.Layout.Row = 2;
            app.VoltageDropDown.Layout.Column = 1;
            app.PickButton = uibutton(controls, 'Text', '在曲线上选区间', ...
                'Enable', 'off', 'Tag', 'PickRange', ...
                'ButtonPushedFcn', @(~, ~) app.togglePicking());
            app.PickButton.Layout.Row = 2;
            app.PickButton.Layout.Column = 2;
            clearButton = uibutton(controls, 'Text', '清空所有区间', ...
                'ButtonPushedFcn', @(~, ~) app.clearRanges());
            clearButton.Layout.Row = 3;
            clearButton.Layout.Column = [1 2];
            app.FitButton = uibutton(controls, 'Text', '拟合  y = kx + b', ...
                'FontWeight', 'bold', 'Enable', 'off', 'Tag', 'FitVoltage', ...
                'ButtonPushedFcn', @(~, ~) app.performFit());
            app.FitButton.Layout.Row = 4;
            app.FitButton.Layout.Column = [1 2];
            app.StatusLabel = uilabel(root, 'Text', '就绪。无需额外 MATLAB 工具箱。', ...
                'WordWrap', 'on', 'Interpreter', 'none', 'Tag', 'MainStatus');
            app.StatusLabel.Layout.Row = 5;
        end

        function chooseCsv(app)
            [name, folder] = uigetfile('*.csv', '选择 SensorWaveformViewer 导出的 CSV');
            if isequal(name, 0)
                return
            end
            try
                app.loadCsv(fullfile(folder, name));
            catch exception
                uialert(app.MainFigure, exception.message, '读取失败');
            end
        end

        function drawWaveforms(app)
            cla(app.WaveAxes);
            app.RangeGraphics = gobjects(0);
            hold(app.WaveAxes, 'on');
            for sensor = 1:height(app.Data.Sensors)
                selected = app.Data.SensorIndex == sensor;
                plot(app.WaveAxes, app.Data.Time_s(selected), app.Data.Values(selected), ...
                    'Color', app.Colors(sensor, :), 'LineWidth', 1.1, ...
                    'DisplayName', app.Data.Sensors.Label(sensor), ...
                    'HitTest', 'off', 'PickableParts', 'none');
            end
            hold(app.WaveAxes, 'off');
            xlabel(app.WaveAxes, '相对时间 / s');
            ylabel(app.WaveAxes, '锁相输出（CSV 原值）');
            title(app.WaveAxes, '各传感器锁相输出');
            grid(app.WaveAxes, 'on');
            legend(app.WaveAxes, 'show', 'Location', 'best', 'Interpreter', 'none');
            app.resetView();
        end

        function resetView(app)
            if isempty(app.Data)
                return
            end
            axis(app.WaveAxes, 'tight');
            limits = [min(app.Data.Time_s), max(app.Data.Time_s)];
            if limits(1) == limits(2)
                limits = limits + [-0.5, 0.5];
            end
            xlim(app.WaveAxes, limits);
            yLimits = [min(app.Data.Values), max(app.Data.Values)];
            margin = max(0.05 * diff(yLimits), 0.01 * max(1, max(abs(yLimits))));
            ylim(app.WaveAxes, yLimits + [-margin, margin]);
            app.drawRanges();
        end

        function togglePicking(app)
            if app.Picking
                app.stopPicking();
                app.StatusLabel.Text = '已取消本次选择，原有区间保留。';
                return
            end
            app.Picking = true;
            app.SelectionStart = NaN;
            app.PickButton.Text = '取消选择 (Esc)';
            zoom(app.MainFigure, 'off');
            pan(app.MainFigure, 'off');
            datacursormode(app.MainFigure, 'off');
            disableDefaultInteractivity(app.WaveAxes);
            app.WaveAxes.Toolbar.Visible = 'off';
            app.MainFigure.Pointer = 'crosshair';
            app.StatusLabel.Text = sprintf('正在选择 %d V：请点击稳定区间的起点。', ...
                app.Voltages(app.VoltageDropDown.Value));
        end

        function waveClicked(app)
            if ~app.Picking || ~strcmp(app.MainFigure.SelectionType, 'normal')
                return
            end
            point = app.WaveAxes.CurrentPoint;
            time = point(1, 1);
            if time < min(app.Data.Time_s) || time > max(app.Data.Time_s)
                return
            end
            if isnan(app.SelectionStart)
                app.SelectionStart = time;
                app.CursorGraphics = xline(app.WaveAxes, time, '--k', '起点', ...
                    'HandleVisibility', 'off', 'HitTest', 'off', 'PickableParts', 'none');
                app.StatusLabel.Text = sprintf('起点 %.6g s；请点击终点。按 Esc 可取消。', time);
            else
                interval = sort([app.SelectionStart, time]);
                if interval(1) == interval(2)
                    app.StatusLabel.Text = '起点和终点不能相同，请重新点击终点。';
                    return
                end
                row = app.VoltageDropDown.Value;
                app.Ranges(row, :) = interval;
                app.stopPicking();
                app.invalidateFit();
                app.RangeTable.Data = [app.Voltages, app.Ranges];
                app.drawRanges();
                next = find(any(isnan(app.Ranges), 2), 1);
                if isempty(next)
                    app.StatusLabel.Text = '五档区间均已选择。检查稳定段后，点击“拟合”。';
                else
                    app.VoltageDropDown.Value = next;
                    app.StatusLabel.Text = sprintf('已设置 %d V。下一档 %d V，点击“在曲线上选区间”继续。', ...
                        app.Voltages(row), app.Voltages(next));
                end
            end
        end

        function stopPicking(app)
            app.Picking = false;
            app.SelectionStart = NaN;
            delete(app.CursorGraphics(isgraphics(app.CursorGraphics)));
            app.CursorGraphics = gobjects(0);
            if ~isempty(app.MainFigure) && isvalid(app.MainFigure)
                app.MainFigure.Pointer = 'arrow';
                app.PickButton.Text = '在曲线上选区间';
                enableDefaultInteractivity(app.WaveAxes);
                app.WaveAxes.Toolbar.Visible = 'on';
            end
        end

        function keyPressed(app, event)
            if strcmp(event.Key, 'escape') && app.Picking
                app.stopPicking();
                app.StatusLabel.Text = '已取消本次选择，原有区间保留。';
            end
        end

        function rangeEdited(app, event)
            value = event.NewData;
            if ~isnumeric(value) || ~isscalar(value) || ~isreal(value) || isinf(value)
                app.RangeTable.Data = [app.Voltages, app.Ranges];
                uialert(app.MainFigure, '请输入有限的秒数；NaN 表示尚未选择。', '无效时间');
                return
            end
            app.stopPicking();
            app.Ranges(event.Indices(1), event.Indices(2) - 1) = value;
            app.invalidateFit();
            app.drawRanges();
            app.StatusLabel.Text = '区间已修改。请点击“拟合”重新计算。';
        end

        function clearRanges(app)
            app.setRanges(nan(5, 2));
            app.VoltageDropDown.Value = 1;
            app.StatusLabel.Text = '区间已清空。请从 100 V 开始选择。';
        end

        function drawRanges(app)
            delete(app.RangeGraphics(isgraphics(app.RangeGraphics)));
            app.RangeGraphics = gobjects(0);
            if isempty(app.Data)
                return
            end
            shades = lines(5);
            for row = 1:5
                if all(isfinite(app.Ranges(row, :))) && app.Ranges(row, 1) < app.Ranges(row, 2)
                    % xline 不改变输出曲线的量程，边界标签明确标出每个电压区间。
                    a = xline(app.WaveAxes, app.Ranges(row, 1), '--', ...
                        sprintf('%d V 起', app.Voltages(row)), ...
                        'Color', shades(row, :), 'HandleVisibility', 'off', ...
                        'HitTest', 'off', 'PickableParts', 'none');
                    b = xline(app.WaveAxes, app.Ranges(row, 2), ':', ...
                        sprintf('%d V 止', app.Voltages(row)), ...
                        'Color', shades(row, :), 'HandleVisibility', 'off', ...
                        'HitTest', 'off', 'PickableParts', 'none');
                    app.RangeGraphics = [app.RangeGraphics; a; b];
                end
            end
        end

        function invalidateFit(app)
            app.Results = table();
            app.Points = table();
            if ~isempty(app.FitFigure) && isvalid(app.FitFigure)
                delete(app.FitFigure);
            end
        end

        function performFit(app)
            try
                app.runFit();
            catch exception
                uialert(app.MainFigure, exception.message, '无法拟合');
            end
        end

        function showFitWindow(app)
            if ~isempty(app.FitFigure) && isvalid(app.FitFigure)
                delete(app.FitFigure);
            end
            app.FitFigure = uifigure('Name', '各传感器的电压线性拟合', ...
                'Position', [120 90 1220 800], 'Tag', 'SensorVoltageFit');
            root = uigridlayout(app.FitFigure, [4 1]);
            root.RowHeight = {44, '1x', 185, 40};
            top = uigridlayout(root, [1 2]);
            top.Layout.Row = 1;
            top.Padding = [0 0 0 0];
            top.ColumnWidth = {'1x', 210};
            uilabel(top, 'Text', 'x：施加电压 / V    y：锁相均值    圆点及误差棒：均值 ± 标准差    实线：拟合', ...
                'FontSize', 14);
            uibutton(top, 'Text', '保存拟合系数 CSV', 'Tag', 'SaveCoefficients', ...
                'ButtonPushedFcn', @(~, ~) app.chooseCoefficientFile());
            plotPanel = uipanel(root, 'Scrollable', 'on', 'BorderType', 'none');
            plotPanel.Layout.Row = 2;
            count = height(app.Results);
            columns = min(count, 2);
            rows = ceil(count / columns);
            plots = uigridlayout(plotPanel, [rows, columns]);
            plots.Scrollable = 'on';
            plots.RowHeight = repmat({295}, 1, rows);
            plots.ColumnWidth = repmat({'1x'}, 1, columns);
            for sensor = 1:count
                ax = uiaxes(plots, 'Tag', sprintf('SensorFit%d', sensor));
                ax.Layout.Row = ceil(sensor / columns);
                ax.Layout.Column = mod(sensor - 1, columns) + 1;
                points = app.Points(app.Points.Order == sensor, :);
                valid = points.Count > 0;
                hold(ax, 'on');
                errorbar(ax, points.Voltage_V(valid), points.Mean(valid), points.Std(valid), ...
                    'o', 'LineStyle', 'none', 'Color', app.Colors(sensor, :), ...
                    'MarkerFaceColor', app.Colors(sensor, :), 'DisplayName', '均值 ± 标准差');
                fit = app.Results(sensor, :);
                if isfinite(fit.k)
                    x = linspace(100, 1000, 100);
                    plot(ax, x, fit.k .* x + fit.b, '-', 'LineWidth', 1.5, ...
                        'Color', app.Colors(sensor, :), 'DisplayName', '线性拟合');
                    detail = sprintf('y = %.6g x %+.6g   |   R² = %.6f   r = %.6f', ...
                        fit.k, fit.b, fit.R2, fit.r);
                else
                    detail = char(fit.Status);
                end
                title(ax, {char(app.Data.Sensors.Label(sensor)), detail}, ...
                    'Interpreter', 'none', 'FontSize', 11);
                xlabel(ax, '施加电压 / V');
                ylabel(ax, '锁相输出（CSV 原值）');
                xticks(ax, app.Voltages);
                xlim(ax, [50 1050]);
                grid(ax, 'on');
                % The shared caption explains both marks; avoid floating legends in a scrollable grid.
                hold(ax, 'off');
            end
            resultsTable = uitable(root, 'Data', app.Results, 'RowName', [], ...
                'ColumnEditable', false, 'Tag', 'FitResults', ...
                'ColumnName', {'顺序', 'NodeId', '通道', 'k（输出/V）', 'b（输出）', ...
                'R²', 'r', 'RMSE', '状态'}, ...
                'ColumnWidth', {50, 90, 60, 125, 125, 95, 95, 95, 'auto'});
            resultsTable.Layout.Row = 3;
            successful = sum(isfinite(app.Results.k));
            app.FitStatusLabel = uilabel(root, 'WordWrap', 'on', 'Interpreter', 'none', ...
                'Text', sprintf('已拟合 %d/%d 个传感器。按通道编号、NodeId 排序保存；缺档传感器保留行并标记 NaN。', ...
                successful, count));
            app.FitStatusLabel.Layout.Row = 4;
        end

        function chooseCoefficientFile(app)
            [folder, name] = fileparts(app.Data.SourceFile);
            [file, destination] = uiputfile('*.csv', '保存所有传感器的拟合系数', ...
                fullfile(folder, name + "_fit_coefficients.csv"));
            if isequal(file, 0)
                return
            end
            [~, ~, extension] = fileparts(file);
            if isempty(extension)
                file = [file, '.csv'];
            end
            try
                filePath = fullfile(destination, file);
                app.saveCoefficients(filePath);
                app.FitStatusLabel.Text = "已保存：" + string(filePath);
            catch exception
                uialert(app.FitFigure, exception.message, '保存失败');
            end
        end
    end
end
