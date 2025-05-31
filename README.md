# ros_end 项目

这是一个 ROS 项目，旨在开发一个能够在模拟环境 (Gazebo) 和真实环境中完成图书馆相关工作的机器人。

项目目前包含以下主要功能包：

*   `gazebosim_demo`: Gazebo 仿真环境和相关演示。
*   `navigation`: 机器人的导航功能实现。
*   `realbot_demo`: 真实机器人平台的接口和演示。

## 协作开发规范

*   **代码同步**: 请使用标准的 `git pull` 获取最新代码，使用 `git push` 推送你的本地提交。
    *   提交信息应简洁明了，描述所做的更改。
    *   提交信息格式建议为：`[功能/修复] 描述`，例如 `[修复] 修复导航中的一个bug`。
    *   可以使用 `git fetch` 和 `git rebase` 来保持本地分支与远程分支的同步，但是使用之前请**确保你了解其工作原理**。
*   **分支管理**: (可选，如果需要可以添加) 建议为新功能或修复创建单独的分支，完成后合并回主分支。
*   **禁止上传生成文件**: 请确保不要将 ROS 编译生成的文件（如 `build/`, `devel/` 目录下的内容）、地图文件 (`.pgm`, `.yaml`) 或其他运行时产生的数据文件提交到 Git 仓库。
    *   项目根目录下的 `.gitignore` 文件已配置忽略常见生成文件，请勿修改。
    *   请不要随意修改其他协作者的代码，除非你们已经讨论过。

## 为脚本添加执行权限

如果你的代码中包含需要直接执行的脚本文件（例如 `.sh` 或 `.py` 文件），请确保通过 Git 为其添加执行权限：

```bash
git update-index --add --chmod=+x <your_script_name>
```

将 `<your_script_name>` 替换为你的脚本文件名。执行此命令后，再进行 `git commit` 和 `git push`，这样其他协作者拉取代码后，该脚本将自动拥有执行权限。

## 构建与运行

请遵循标准的 ROS 构建和运行流程：

```bash
# 切换到工作空间根目录
cd /path/to/your/ros_end_workspace 

# 构建
catkin_make

# 设置环境 (每次打开新终端都需要执行)
source devel/setup.bash

# 运行示例 (请根据实际情况替换包名和启动文件名)
# 例如，启动 Gazebo 仿真环境:
# roslaunch gazebosim_demo demo.launch 
# 例如，启动导航:
# roslaunch navigation navigation.launch
# 例如，启动真实机器人:
# roslaunch realbot_demo realbot.launch
```

如果你已经将其他workspace的setup.bash文件添加到你的.bashrc中，可以采用如下的办法避免每次都要source或者覆盖了之前的source：
1. 给第一个工作空间生成install空间，作为`Underlay`
```bash
cd ~/catkin_ws
catkin_make install
```

2. 修改`~/.bashrc`
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/install/setup.bash
source /path/to/your/ros_end_workspace/devel/setup.bash
```