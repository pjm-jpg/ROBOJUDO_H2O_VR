# 从 RoboJuDo 根目录统一提交

依赖继续使用 Git 子模块；本地改动以完整补丁保存在主仓库的 `third_party/patches/`。不需要分别给 Viewer、PHC 等建立个人分支或推送依赖仓库。

## 日常只用一个提交命令

```bash
cd /home/pjm/Desktop/RoboJuDo
python scripts/commit_all.py -m "更新 VR 遥操与仿真"
git push
```

脚本先将依赖的本地提交、未提交修改、新增文件和删除保存为补丁，再暂存项目所有未被 `.gitignore` 排除的文件，最后创建一次主仓库提交。它不会推送远程；提交前应确认项目里没有不想一起保存的文件。被忽略的环境、缓存、日志和动作数据等不会自动纳入 Git。

如果先看内容再提交：

```bash
python scripts/commit_all.py --stage-only
git diff --cached --stat
git diff --cached
git commit -m "更新 VR 遥操与仿真"
git push
```

补丁快照保存当前工作树的最终内容，等同于把依赖内的改动全部暂存，而非只保存依赖原来已暂存的部分。同步过程使用临时索引和对象目录，不改依赖自己的索引或 HEAD。

## 自动同步与版本固定

本机启用 `.githooks/pre-commit` 后，正常主仓库提交也会刷新并暂存依赖补丁。只修改依赖时，Git 的空提交预检查可能要求再提交一次，因此推荐始终使用上面的 `commit_all.py`，它在 Git 检查前就完成同步。钩子也会将全部依赖差异一起纳入提交，不适合拆分依赖内部的部分改动。

```bash
# 每份新克隆只需执行一次；已有其他钩子时会停止并保留原配置。
python scripts/sync_submodule_patches.py --install-hook

# 只刷新补丁，不创建提交，也不暂存普通主仓库文件。
python scripts/sync_submodule_patches.py
```

`third_party/patches/local_sources.json` 固定四个依赖的公开上游基准版本。补丁始终相对这些版本生成，主仓库的子模块指针也保持这些版本，避免引用仅在本机存在的提交。未来有意升级上游时，应同时审查基准、补丁和安装配置。子模块没有初始化时保留已保存的补丁；不会把缺少的目录误认为删除源码。

当前保存：

- `mujoco_viewer.patch`：Viewer 的完整本地差异，包含可视化扩展与相机回调兼容修改。
- `phc.patch`：PHC 的完整本地差异，包含修改的工具代码、G1 配置/XML 和 Python 打包文件。
- `unitree_cpp.patch`、`zed_proxy.patch`：目前为空，未来修改也由同一流程捕获。

编辑器仍可能把这些目录显示成独立 Git 仓库；它们在磁盘上的仓库结构没有变化。统一提交由补丁同步完成。 主仓库将这四个子模块设为 `ignore = all`，避免本地依赖 HEAD 与固定上游版本不同导致主仓库提交后仍显示一个未提交指针。源码变化通过同步后的补丁呈现；仅修改依赖时请运行统一提交脚本，或先手动刷新补丁。

## 另一台电脑还原依赖

```bash
git clone --recurse-submodules https://github.com/pjm-jpg/ROBOJUDO_H2O_VR.git
cd ROBOJUDO_H2O_VR
# 先按项目说明创建环境并安装主项目，再执行：
python submodule_install.py mujoco_viewer phc
python scripts/sync_submodule_patches.py --install-hook
```

安装器初始化尚未下载的子模块，再应用主仓库补丁。已经应用过的补丁会跳过；遇到本地修改冲突会停止，不执行强制重置或清理。PHC 的 G1 新增文件已并入完整补丁，默认配置不再重复复制旧 addon 目录。

只还原源码、不安装 Python 包时使用：

```bash
python submodule_install.py mujoco_viewer phc --source-only
```

机器人 SDK 等额外系统依赖仍按原安装说明配置。此流程只管理 RoboJuDo 仓库及其四个子模块；外部 H2O-VR 项目、被忽略的动作文件和环境安装不在本次 Git 快照范围内。

## 验证

已从四个依赖的固定上游版本分别重建源码并应用补丁，逐文件比较内容、文件集合、符号链接和可执行位，与本机当前源码一致；重复应用安全。测试也覆盖未推送的依赖提交、二进制新增文件、删除、只改依赖时的一次根目录提交，以及冲突时保留本地修改。
