## Chinese
<details open>
<summary>🇨🇳 中文说明</summary>
🛠️ 嵌入式端移植与配置步骤（STM32F446）

请严格按以下步骤操作，确保项目能正确生成并编译：

1. 使用 Git 克隆项目（不要复制粘贴！）
```bash
git clone https://github.com/yourname/autonomous-car-project.git
cd autonomous-car-project/embedded
```

2. 安装必要工具
安装 [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)
安装 [Keil MDK-ARM](https://www2.keil.com/mdk5/)（建议 v5.38 或更高）

3. 生成 Keil 工程
用 STM32CubeMX 打开 MPU6050.ioc
点击菜单栏 Project → Generate Code
等待代码生成完成（即使提示工程生成问题，也请继续下一步）

4. 编译项目
打开生成的 MDK-ARM/MPU6050.uvprojx
确保 APP/ 文件夹已添加到包含路径：
Options for Target → C/C++ → Include Paths → 添加 ..\APP
点击 Build，应无编译错误

5. （可选）使用 STM32CubeIDE
在 CubeMX 中将工具链改为 STM32CubeIDE
重新生成代码，直接导入 IDE 即可编译
✅ 提示：始终通过 git clone 获取项目，切勿手动复制文件夹。

</details>

## English

<details>
<summary>🇺🇸 English</summary>
🛠️ Embedded Porting & Setup Instructions (STM32F446)

Follow these steps exactly to ensure successful code generation and compilation:

1. Clone the project using Git (do NOT copy-paste!)
```bash
git clone https://github.com/yourname/autonomous-car-project.git
cd autonomous-car-project/embedded
```

2. Install required tools
Install [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)
Install [Keil MDK-ARM](https://www2.keil.com/mdk5/) (v5.38 or later recommended)

3. Generate the Keil project
Open MPU6050.ioc in STM32CubeMX
Click Project → Generate Code
Wait for generation to complete (proceed even if a project-generation warning appears)

4. Build the project
Open the generated MDK-ARM/MPU6050.uvprojx
Add the APP/ folder to include paths:
Options for Target → C/C++ → Include Paths → Add ..\APP
Click Build — compilation should succeed with no errors

5. (Optional) Use STM32CubeIDE
In CubeMX, switch the toolchain to STM32CubeIDE
Regenerate code and import directly into the IDE
✅ Tip: Always use git clone to obtain the project—never manually copy the folder.

</details>
