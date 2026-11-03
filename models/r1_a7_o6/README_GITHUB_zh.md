# R1-A7 + 双 O6 灵巧手（独立上部模型）

从用户上传的 `r1_a7_o6_model(1).zip` 引入。保留 XML、双手描述和示例 Python 脚本；将文件名统一为 `r1_a7_o6.xml` 和 `scene_o6.xml`，并修复场景 include 引用。

资源：18 个上部 STL、48 个 O6 手部 OBJ，需使用 `scripts/import_uploaded_archives.py` 从原 ZIP 导入到当前目录。

模型中的上部躯干/双臂与双 O6 手部关节总数为 39（17 + 22），而**R1-A7 的这版模型没有直接融合进 AS2-W 的 33 轴整机 XML**；其 XML 执行器/附属被动关节配置不能简单用关节数推断，应以 MuJoCo 加载结果为准。

验证（恢复资源后）：

```bash
python scripts/check_uploaded_models.py --strict --compile
python models/r1_a7_o6/demo.py
```

`demo.py` 的运行目录/加载行为以源脚本为准；建议从 `models/r1_a7_o6/` 启动。独立上部模型有 39 个真实关节（17+22）、29 个执行器；手部包括位置控制和耦合关节，**不等于 39 个独立力矩电机**。

源压缩包中的网络、版权和发布条款并未明确随附。本仓库默认不公开重新分发 STL/OBJ，待取得相关权限后再调整。
