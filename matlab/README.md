# MATLAB 多传感器曲线与电压标定

读取 SensorWaveformViewer 保存的锁相输出 CSV，在一张图中叠加所有传感器的时间曲线，再将 **100、200、300、500、1000 V** 五档电压的稳定输出均值拟合为 `y = kx + b`。

## 启动

将 MATLAB 当前文件夹切换到本目录，运行 **`sensor_voltage_viewer.m`**，或在命令窗口输入：

```matlab
SensorVoltageApp
```

也可以直接指定采集文件：

```matlab
app = SensorVoltageApp("D:\data\multi_dc_sensor_data.csv");
```

使用基础 MATLAB 即可，不需要 Curve Fitting Toolbox、Statistics and Machine Learning Toolbox 或 Image Processing Toolbox。本工具面向本机已安装的 MATLAB R2025a。

## 操作顺序

1. 点击 **打开 CSV**，选择上位机“导出多节点 CSV”保存的文件。每个 NodeId 对应一条曲线，图例同时显示 CH 和 NodeId。
2. 在电压下拉框选择 **100 V**，点击 **在曲线上选区间**，依次点击该电压稳定段的起点和终点。可先使用坐标轴工具栏缩放曲线，再进入选区模式；选区时暂停缩放/平移，按 Esc 取消。
3. 按相同方法依次选择 **200、300、500、1000 V**。五个区间必须按时间递增且不重叠。曲线上的竖线标出边界；下方表格可以精确修改开始、结束秒数。NaN 表示尚未设置。
4. 点击 **拟合 y = kx + b**。程序对每个传感器分别计算各档区间均值，然后在**独立窗口**中绘制五个标定点、标准差误差棒和拟合直线，显示 **k、b、R²、r、RMSE**。传感器较多时，可滚动浏览所有拟合图。
5. 在拟合窗口点击 **保存拟合系数 CSV**，选择保存位置。所有传感器一次保存，每个传感器一行，按 CH 数字升序、同 CH 内按 NodeId 数值升序排列。没有有效 CH 的传感器排在最后。

主窗口重新读取文件、修改或清空区间后，旧拟合结果及窗口会清除，需要重新拟合，避免保存过期系数。关闭拟合窗口不会关闭曲线窗口；关闭主窗口会同时关闭拟合窗口。

## CSV 配套规则

支持上位机现有两种实时数据格式（UTF-8，包括 BOM）：

```text
node_id,channel,timestamp,elapsed_s,seq,dac,lockin,adc,display_field,display_value
timestamp,elapsed_s,seq,dac,lockin,adc
```

- 始终读取 **`lockin`**；保留上位机保存的小数和缩放，不再次除以 100。`dac`、`adc`、`display_value` 都不会被当成施加电压或锁相输出。
- 多节点传感器用 **NodeId** 区分。上位机 CH 位置可能被新设备复用，同一个 CH 下的不同 NodeId 仍独立绘图、拟合、保存。同一 NodeId 出现多个 CH 时合并，并提示；排序使用其最小有效 CH。
- 横轴优先使用 CSV 的 **`elapsed_s`**，不会为不同传感器分别重新计时。无此列时由 `timestamp` 计算相对秒数；少量缺失时间在有共同参考时使用 `timestamp` 恢复，并在状态栏说明。
- 非有限输出、无效时间或无效 NodeId 按行忽略，并给出数量。不会因为一个传感器缺数就删除其他传感器的数据。
- DAC 扫描 CSV 没有实时数据时间轴，一对一交流原始 ADC CSV 没有 `lockin`，均会提示不能用于本工具。
- CSV 不含外加电压标签，五档电压对应关系来自你选择的稳定区间。请避开切换瞬间和未稳定部分，保证所选区间对各传感器都适用。

## 拟合和保存内容

`x` 是施加电压（V），`y` 是锁相输出的区间均值。每个电压档一个点，五档**等权**拟合，采样点较多的电压档不会增加拟合权重。前四档使用 `[开始,结束)`，第五档使用 `[开始,结束]`，相邻区间共用边界时不会重复计数。

- `k`：斜率，单位为 CSV 锁相输出单位/V；`b`：截距，单位同锁相输出。上位机没有为 `lockin` 标注物理单位，因此界面明确显示“CSV 原值”。
- `R2`：决定系数 `1 - SSE/SST`；`r`：Pearson 相关系数；`RMSE`：五个区间均值相对拟合直线的均方根误差。
- 某传感器任一电压档没有有效样点时，不进行不完整拟合；保留该行，系数为 `NaN`，`Status` 指出缺少的电压档。
- 五档均值完全相同时，`k=0`、`b=常量`，`R2` 和 `r` 为 `NaN`，状态说明其未定义。
- 误差棒表示区间内样本标准差，并非拟合置信区间。

导出列为：

```text
Order,NodeId,Channel,k,b,R2,r,RMSE,Status
```

保存会保留全部传感器的顺序和身份，也保留失败状态；不会用系数表覆盖原始采集 CSV。

## 程序化使用和模拟演示

已知稳定段时间时，不必逐次点击：

```matlab
data = read_sensor_waveform_csv("D:\data\record.csv");
ranges = [2 10; 14 22; 26 34; 38 46; 50 58]; % 示例，需改成实际稳定区间
[coefficients, points] = fit_sensor_voltage(data, ranges);
writetable(coefficients, "coefficients.csv", 'Encoding', 'UTF-8');
```

`points` 包含每个传感器、每个电压档的时间范围、样本数、均值、标准差，可用于复核选区。

无需真实数据的演示（生成的是**人工模拟数据**）：

```matlab
[file, ranges] = create_demo_sensor_csv;
app = SensorVoltageApp(file);
app.setRanges(ranges);
app.runFit();
```

演示中三个传感器的真实 `(k,b)` 分别为 `(0.50,10)`、`(1.20,20)`、`(-0.10,130)`。

运行验证：

```matlab
results = runtests(fullfile(pwd, 'tests'));
assertSuccess(results);
```

覆盖 CSV 格式和精度、NodeId/CH 复用、时间恢复、异常数据、五档拟合、等权统计、区间边界、常量输出、缺档提示、独立图窗、保存顺序及原数据保护。

## 本次验证状态

2026-09-20：已在本机 MATLAB R2025a 中运行全部 **30 项测试，30 通过、0 失败、0 未完成**。覆盖 CSV 解析、五档拟合、单传感器和多传感器、真实鼠标依次选区并点击拟合、系数保存顺序与数值、Esc 取消、结果失效处理及原始文件保护。使用的是按上位机格式生成的模拟 CSV；尚未提供真实采集文件。

所有 8 个 `.m` 文件经过 `mlint` 语法检查，无语法错误。图窗已实际生成并检查布局；窗口测试会更新本目录下的预览：

- `preview_waveforms.png`：三个模拟传感器的输出曲线与五档选区。
- `preview_fits.png`：独立拟合窗口及系数表，图区域支持滚动。
- `validation.log`：完整测试结果。

测试会生成临时 CSV，并在每项测试结束后逐个清理；不会修改或删除原始采集数据。