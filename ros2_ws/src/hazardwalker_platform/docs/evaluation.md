# 结果格式与评估方法

本文说明参赛算法输出格式，以及评估脚本如何计算客观指标。

## 输出文件

参赛算法完成探索和识别后，应生成：

```text
results/detected_danger.json
```

格式如下：

```json
{
  "exploration_time": 98.76,
  "detected_danger_sources": [
    {"position": [2.34, -1.56, 0.25]},
    {"position": [-3.21, 4.78, 1.75]}
  ]
}
```

字段说明：

- `exploration_time`：探索耗时，单位秒。
- `detected_danger_sources`：识别出的危险源列表。
- `position`：危险源在 Gazebo `world` 坐标系下的位置，三维坐标 `[x, y, z]`，单位 m。

## 评估命令

```bash
python3 ./src/building_obstacles/scripts/evaluate_danger.py \
  --truth-file ./results/danger_truth.json \
  --detected-file ./results/detected_danger.json \
  --output-file ./results/evaluation_result.json
```

旧拼写脚本仍保留兼容：

```bash
python3 ./src/building_obstacles/scripts/evaulate_danger.py
```

## 匹配规则

默认匹配阈值为 `1.0 m`。评估脚本会对每个真实危险源和每个检测点计算三维欧氏距离。距离小于等于阈值的组合进入候选集。

候选集按距离从小到大排序，然后进行一对一贪心匹配：

1. 从最近的一对开始处理。
2. 如果该真值点和检测点都未被匹配，则接受这对匹配。
3. 如果任意一方已经被匹配，则跳过。
4. 每个真值点最多被匹配一次，每个检测点也最多被匹配一次。

匹配完成后：

- `correct`：成功匹配到的真实危险源数量。
- `missed`：没有被匹配到的真实危险源数量。
- `false_alarms`：没有匹配到任何真实危险源的检测点数量。

## 识别概率

当前脚本使用的识别指标更接近召回率：

```text
prob = correct / truth_count
```

得分规则：

- `truth_count == 0`：0 分。
- `prob <= 0.6`：0 分。
- `prob > 0.6`：`14 * prob`。

识别概率满分为 14 分。

## 虚警率

虚警率计算方式：

```text
far = false_alarms / detected_count
```

分母是选手提交的检测点总数，不是真实危险源总数。

得分规则：

- `detected_count == 0`：0 分。
- `far <= 0.1`：8 分。
- 超过 10% 后，每多 5% 扣 1 分。
- 最低 0 分。

虚警率满分为 8 分。

## 探索时间

探索时间得分满分 15 分：

- `exploration_time <= 600` 秒：15 分。
- 超过 600 秒后，每满 60 秒扣 1 分。
- 最低 0 分。

## 客观总分

```text
technical_objective_total =
  exploration_time_score
  + recognition_probability_score
  + false_alarm_rate_score
```

客观部分最高 37 分。

评估结果写入：

```text
results/evaluation_result.json
```
