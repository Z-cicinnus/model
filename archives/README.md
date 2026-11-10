# 完整历史更新包（分块保留原始字节）

为保留每一版源码、数据及图像，本仓库存放三份原 ZIP 的**逐字节分块归档**，每块最多 2 MiB，避免 GitHub 普通 Git 单文件上限。

| 名称 | 目录 | 来源 |
|---|---|---|
| v2 动态基准 | `packages/dynamic_v2/` | `Centaur_Dynamic_Benchmark_v2.zip` |
| v5 固定协议基准 | `packages/baseline_v5_100hz/` | `Centaur_Algorithm_Baseline_v5_100Hz.zip` |
| v6 WBC | `packages/wbc_v6_100hz/` | `Centaur_WBC_v6_100Hz.zip` |

原版 ZIP 的大小、SHA-256、每个分块 SHA-256 和 Git blob SHA 均记录在 `manifest.json`。

恢复全部归档：

```bash
python scripts/reconstruct_update_packages.py --all --output-dir update_packages
```

仅恢复最新版：

```bash
python scripts/reconstruct_update_packages.py --version wbc_v6_100hz --output-dir update_packages
```

脚本会逐块与整体核验 SHA-256，缺块和哈希不匹配都会失败，不会将已校验 ZIP 自动解压或覆盖现有文件。

请先完成 Git clone 后再运行。历史成绩在 `benchmarks/` 为方便浏览的**小型摘要**；全部 CSV/JSONL 原始轨迹保存在各自归档 ZIP 内。
