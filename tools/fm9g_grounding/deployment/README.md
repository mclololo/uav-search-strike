# FCOS 部署检测头

`fcos-recovered-20261008/epoch-04/head.safetensors` 于 2026-10-09 从当前云端部署目录取回。它只包含 FCOS 分类/回归头和特征适配层，不包含九格基础视觉权重，也不是 YOLO 模型。

- 大小：29,608,860 字节。
- SHA-256：`7a7cd85a5152a4b354efd134e7126c7e8b0732dbe2fe609f805ebe7aad2fa882`。
- 基础模型：服务器预装的 `/home/ubuntu/model/FM9G4B-V`。
- 输入尺寸：448；分数阈值：0.43；四类顺序以 `grounding_data.CLASSES` 为准。
- 原始部署元数据保留在同目录的 `deployment.json`；其 `test_metrics` 是来源服务器的历史路径，推理不读取该文件。

文件由 Git LFS 管理，clone 后执行 `git lfs pull`。不要把 LFS 文本指针当作权重上传。使用 `sha256sum`（Windows 用 `Get-FileHash`）核验实文件；后续训练头放到新的版本目录，通过 `FM9G_HEAD` 指定，不覆盖该版本。

恢复后的独立测试 Precision 89.90%、Recall 84.36%、F1@IoU0.5 87.04%，平均离线耗时 27.96 ms，p95 30.84 ms；这些是 2026-10-08 重建模型的已记录结果，本次仅核验来源和哈希，没有重新完成飞行验收。部署步骤见 [GPUFree 文件同步说明](../../../docs/GPUFree文件同步说明.md)。
