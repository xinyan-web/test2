# AAE5303 Environment Setup Report — Template for Students

> **Important:** Follow this structure exactly in your submission README.  
> Your goal is to demonstrate **evidence, process, problem-solving, and reflection** — not only screenshots.

---

## 1. System Information

**Laptop model:**

ASUS TUF Gaming A15 FA506IU

**CPU / RAM:**  
AMD Ryzen 5 4600H with Radeon Graphics, 16GB RAM]_

**Host OS:**  
 Ubuntu 22.04

**Linux/ROS environment type:**  
_[Choose one:]_
- [ ] Dual-boot Ubuntu
- [ ] WSL2 Ubuntu
- [√] Ubuntu in VM (UTM/VirtualBox/VMware/Parallels)
- [ ] Docker container
- [ ] Lab PC
- [ ] Remote Linux server

---

## 2. Python Environment Check

### 2.1 Steps Taken

Describe briefly how you created/activated your Python environment:

**Tool used:**  
_[conda]_

**Key commands you ran:**
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

**Any deviations from the default instructions:**  
_[None]_

### 2.2 Test Results

Run these commands and paste the actual terminal output (not just screenshots):

```bash
python scripts/test_python_env.py
```

**Output:**
```
[(aae5303) xinyan@xinyan-virtual-machine:~/aae5303/PolyU-AAE5303-env-smork-test$ python scripts/test_python_env.py
========================================
AAE5303 Environment Check (Python + ROS)
Goal: help you verify your environment and understand what each check means.
========================================

Step 1: Environment snapshot
  Why: We capture platform/Python/ROS variables to diagnose common setup mistakes (especially mixed ROS env).
Step 2: Python version
  Why: The course assumes Python 3.10+; older versions often break package wheels.
Step 3: Python imports (required/optional)
  Why: Imports verify packages are installed and compatible with your Python version.
Step 4: NumPy sanity checks
  Why: We run a small linear algebra operation so success means more than just `import numpy`.
Step 5: SciPy sanity checks
  Why: We run a small FFT to confirm SciPy is functional (not just installed).
Step 6: Matplotlib backend check
  Why: We generate a tiny plot image (headless) to confirm plotting works on your system.
Step 7: OpenCV PNG decoding (subprocess)
  Why: PNG decoding uses native code; we isolate it so corruption/codec issues cannot crash the whole report.
Step 8: Open3D basic geometry + I/O (subprocess)
  Why: Open3D is a native extension; ABI mismatches can segfault. Subprocess isolation turns crashes into readable failures.
Step 9: ROS toolchain checks
  Why: The course requires ROS tooling. This check passes if ROS 2 OR ROS 1 is available (either one is acceptable).
  Action: building ROS 2 workspace package `env_check_pkg` (this may take 1-3 minutes on first run)...
  Action: running ROS 2 talker/listener for a few seconds to verify messages flow...
Step 10: Basic CLI availability
  Why: We confirm core commands exist on PATH so students can run the same commands as in the labs.

=== Summary ===
✅ Environment: {
  "platform": "Linux-6.8.0-94-generic-x86_64-with-glibc2.35",
  "python": "3.10.19",
  "executable": "/home/xinyan/miniconda3/envs/aae5303/bin/python",
  "cwd": "/home/xinyan/aae5303/PolyU-AAE5303-env-smork-test",
  "ros": {
    "ROS_VERSION": "2",
    "ROS_DISTRO": "humble",
    "ROS_ROOT": null,
    "ROS_PACKAGE_PATH": null,
    "AMENT_PREFIX_PATH": "/opt/ros/humble",
    "CMAKE_PREFIX_PATH": null
  }
}
✅ Python version OK: 3.10.19
✅ Module 'numpy' found (v2.2.6).
✅ Module 'scipy' found (v1.15.3).
✅ Module 'matplotlib' found (v3.10.8).
✅ Module 'cv2' found (v4.13.0).
✅ Module 'rclpy' found (vunknown).
✅ numpy matrix multiply OK.
✅ numpy version 2.2.6 detected.
✅ scipy FFT OK.
✅ scipy version 1.15.3 detected.
✅ matplotlib backend OK (Agg), version 3.10.8.
✅ OpenCV OK (v4.13.0), decoded sample image 128x128.
✅ Open3D OK (v0.19.0), NumPy 2.2.6.
✅ Open3D loaded sample PCD with 8 pts and completed round-trip I/O.
✅ ROS 2 CLI OK: /opt/ros/humble/bin/ros2
✅ ROS 1 tools not found (acceptable if ROS 2 is installed).
✅ colcon found: /usr/bin/colcon
✅ ROS 2 workspace build OK (env_check_pkg).
✅ ROS 2 runtime OK: talker and listener exchanged messages.
✅ Binary 'python3' found at /home/xinyan/miniconda3/envs/aae5303/bin/python3

All checks passed. You are ready for AAE5303 🚀
]
```

```bash
python scripts/test_open3d_pointcloud.py
```

**Output:**
```
[(aae5303) xinyan@xinyan-virtual-machine:~/aae5303/PolyU-AAE5303-env-smork-test$ python scripts/test_open3d_pointcloud.py
ℹ️ Loading /home/xinyan/aae5303/PolyU-AAE5303-env-smork-test/data/sample_pointcloud.pcd ...
✅ Loaded 8 points.
   • Centroid: [0.025 0.025 0.025]
   • Axis-aligned bounds: min=[0. 0. 0.], max=[0.05 0.05 0.05]
✅ Filtered point cloud kept 7 points.
✅ Wrote filtered copy with 7 points to /home/xinyan/aae5303/PolyU-AAE5303-env-smork-test/data/sample_pointcloud_copy.pcd
   • AABB extents: [0.05 0.05 0.05]
   • OBB  extents: [0.08164966 0.07071068 0.05773503], max dim 0.0816 m
(aae5303) xinyan@xinyan-virtual-machine:~/aae5303/PolyU-AAE5303-env-smork-test$ 
]
```

**Screenshot:**  
_[Include one screenshot showing both tests passing]_
![Python Tests Passing](images/jieguo1.png)

---

## 3. ROS 2 Workspace Check

### 3.1 Build the workspace

Paste the build output summary (final lines only):

```bash
source /opt/ros/humble/setup.bash
colcon build
```

**Expected output:**
```
Summary: 1 package finished [x.xx s]
```

**Your actual output:**
```
Summary: 1 package finished [27.6s]

```

### 3.2 Run talker and listener

Show both source commands:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

**Then run talker:**
```bash
ros2 run env_check_pkg talker.py
```

**Output (3–4 lines):**
```
 hello #42'
[INFO] [1769765995.847813914] [env_check_pkg_talker]: Publishing: 'AAE5303 hello #43'
[INFO] [1769765996.348282488] [env_check_pkg_talker]: Publishing: 'AAE5303 hello #44'
[INFO] [1769765996.848563908] [env_check_pkg_talker]: Publishing: 'AAE5303 hello #45'

```

**Run listener:**
```bash
ros2 run env_check_pkg listener.py
```

**Output (3–4 lines):**
```
eard: 'AAE5303 hello #23'
[INFO] [1769766733.901333329] [env_check_pkg_listener]: I heard: 'AAE5303 hello #24'
[INFO] [1769766734.401239076] [env_check_pkg_listener]: I heard: 'AAE5303 hello #25'
[INFO] [1769766734.900751591] [env_check_pkg_listener]: I heard: 'AAE5303 hello #26'
[INFO] [1769766735.401237917] [env_check_pkg_listener]: I heard: 'AAE5303 hello #27'
[INFO] [1769766735.901298704] [env_check_pkg_listener]: I heard: 'AAE5303 hello #28'
[INFO] [1769766736.402134990] [env_check_pkg_listener]: I heard: 'AAE5303 hello #29'
^C[INFO] [1769766742.634929880] [rclcpp]: signal_handler(SIGINT/SIGTERM)
(aae5303) xinyan@xinyan-virtual-machine:~/aae5303/PolyU-AAE5303-env-smork-test$ 

```

**Alternative (using launch file):**
```bash
ros2 launch env_check_pkg env_check.launch.py
```

**Screenshot:**  
_[Include one screenshot showing talker + listener running]_

![Talker and Listener Running](images/listener and talker.png)

---

## 4. Problems Encountered and How I Solved Them

> **Note:** Write 2–3 issues, even if small. This section is crucial — it demonstrates understanding and problem-solving.

### Issue 1: [Write the exact error message or problem]
activate aae5303
activate: command not found


**Cause / diagnosis:**  
The main reason is that activate isn’t a system-executable command, and I used the wrong syntax for virtual environment activation. Also, the virtual environment might not be created, or its path could be incorrect.
Activate is an activation script for Python virtual environments (conda/venv), not a standalone command. It has to be called through the environment’s script path—I can’t just enter activate [environment name]
 


**Fix:**  
_[The exact command/config change you used to solve it]_

```bash
[Your fix command/code here]
cd aae5303/PolyU-AAE5303-env-smork-test/ros2_ws/


**Reference:**  
 _[ AI assistant and Classmate assistant]_




### Issue 2: [Another real error or roadblock]
(aae5303) xinyan@xinyan-virtual-machine:~/aae5303/PolyU-AAE5303-env-smork-test$ ros2 launch env_check_pkg env_check.launch
file 'env_check.launch' was not found in the share directory of package 'env_check_pkg' which is at '/home/xinyan/aae5303/PolyU-AAE5303-env-smork-test/install/env_check_pkg/share/env_check_pkg'



**Cause / diagnosis:**  
The launch file has not been properly installed/compiled into the package path of ROS 2. ROS 2 requires that launch files be configured via CMakeLists.txt or setup.py; only then will they be compiled and installed to the share folder in the install directory — otherwise, the ros2 launch command will fail to detect them.



**Fix:**  


```bash
[Your fix command/code here]
(aae5303) xinyan@xinyan-virtual-machine:~/aae5303/PolyU-AAE5303-env-smork-test/ros2_ws$ source install/setup.bash
ros2 launch env_check_pkg env_check.launch.py
[INFO] [launch]: All log files can be found below /home/xinyan/.ros/log/2026-01-30-18-00-27-959158-xinyan-virtual-machine-71090

```

**Reference:**  
_[Official ROS docs and AI assistant]_

---

### Issue 3 (Optional): [Package 'env_check_pkg' not found]
(aae5303) xinyan@xinyan-virtual-machine:~/aae5303/PolyU-AAE5303-env-smork-test/ros2_ws$ cd ..
(aae5303) xinyan@xinyan-virtual-machine:~/aae5303/PolyU-AAE5303-env-smork-test$ ros2 run env_check_pkg talker
Package 'env_check_pkg' not found



**Cause / diagnosis:**  
The current terminal cannot recognize the compiled path of this ROS 2 package.

**Fix:**  
(aae5303) xinyan@xinyan-virtual-machine:~/aae5303/PolyU-AAE5303-env-smork-test$ source /opt/ros/humble/setup.bash
source install/setup.bash
(aae5303) xinyan@xinyan-virtual-machine:~/aae5303/PolyU-AAE5303-env-smork-test$ ros2 run env_check_pkg talker

**Reference:**  
_[AI assistant]_

---

## 5. Use of Generative AI (Required)

Choose one of the issues above and document how you used AI to solve it.

> **Goal:** Show critical use of AI, not blind copying.

### 5.1 Exact prompt you asked

**Your prompt:**
```
[Copy-paste your actual message to the AI, not a summary]
(aae5303) xinyan@xinyan-virtual-machine:~/aae5303/PolyU-AAE5303-env-smork-test/ros2_ws$ cd ..
(aae5303) xinyan@xinyan-virtual-machine:~/aae5303/PolyU-AAE5303-env-smork-test$ ros2 run env_check_pkg talker
Package 'env_check_pkg' not found

### 5.2 Key helpful part of the AI's answer
你执行ros2 run env_check_pkg talker提示Package 'env_check_pkg' not found，核心原因是当前终端未识别到该 ROS2 包的编译路径，本质是 ROS2 的环境变量未加载 ——ROS2 编译后的包路径、可执行文件路径等信息，都存放在工作空间的install/setup.bash文件中，终端不加载这个文件，就无法找到env_check_pkg包及其中的talker节点。
结合你之前的操作路径（~/aae5303/PolyU-AAE5303-env-smork-test是 ROS2 工作空间根目录），一步就能解决，直接在当前终端执行以下命令加载环境变量：

**AI's response (relevant part only):**
前置条件：工作空间必须先通过colcon build编译，生成install目录后才能加载环境；
# 加载工作空间的ROS2环境变量（你的工作空间根目录路径）
source install/setup.bash
ros2 run env_check_pkg talker
```
[Quote only the relevant part of the AI's answer]
source install/setup.bash
ros2 run env_check_pkg talker
```

### 5.3 What you changed or ignored and why

Explain briefly:
- Did the AI recommend something unsafe?
- Did you modify its solution?
- Did you double-check with official docs?

**Your explanation:**  
The AI’s recommendations were all safe and compliant with standard ROS 2 operation protocols, with no unsafe content involved. I did not modify any of the AI’s proposed solutions and followed them in full during the operation process. I also cross-checked each step with the official ROS 2 documentation to verify the accuracy and rationality of the solutions, ensuring no deviations from official guidelines.

### 5.4 Final solution you applied

Show the exact command or file edit that fixed the problem:
cd ~/aae5303/PolyU-AAE5303-env-smork-test
colcon build
source install/setup.bash
ros2 launch env_check_pkg env_check.launch.py  # 启动launch文件（适配ROS2主流py后缀启动文件）

```bash
[Your final command/code here]
$ cdxinyan@xinyan-virtual-machine:~$ conda
$ conda activate aae5303
$ cd aae5303/PolyU-AAE5303-env-smork-test/ros2_ws/
(aae5303) xinyan@xinyan-virtual-machine:~/aae5303/PolyU-AAE5303-env-smor
k-tsource /opt/ros/humb/opt/ros/humb
le/setup.bash
(aae5303) xinyan@xinyan-virtual-machine:~/aae5303/PolyU-AAE5303-env-smor
k-tcolcon build --eventbuild --event
-handlers console_direct+ --executor sequential
Starting >>> env_check_pkg
[0.9s] [0/1 complete] ...
[ 50%] Built target talker
[100%] Built target listener
Summary: 1 package finished [1.26s]
(aae5303) xinyan@xinyan-virtual-machine:~/aae5303/PolyU-AAE5303-env-smor
k-t```/ros2_ws$ ```
> ros2 run env_check_pkg listener
> ^C
(aae5303) xinyan@xinyan-virtual-machine:~/aae5303/PolyU-AAE5303-env-smor
k-test/ros2_ws$ ```
ros2 run env_check_pkg listener
> ^C
(aae5303) xinyan@xinyan-virtual-machine:~/aae5303/PolyU-AAE5303-env-smork-test/ros2_ws$ ```
ros2 run env_check_pkg listener
> cd ..
> ^C
(aae5303) xinyan@xinyan-virtual-machine:~/aae5303/PolyU-AAE5303-env-smork-test/ros2_ws$ cd ..
(aae5303) xinyan@xinyan-virtual-machine:~/aae5303/PolyU-AAE5303-env-smork-test$ ros2 run env_check_pkg talker
Package 'env_check_pkg' not found
(aae5303) xinyan@xinyan-virtual-machine:~/aae5303/PolyU-AAE5303-env-smork-test$ source /opt/ros/humble/setup.bash
source install/setup.bash
(aae5303) xinyan@xinyan-virtual-machine:~/aae5303/PolyU-AAE5303-env-smork-test$ ros2 run env_check_pkg talker
[INFO] [1769766637.901552970] [env_check_pkg_talker]: AAE5303 talker ready (publishing at 2 Hz).
[INFO] [1769766638.403011510] [env_check_pkg_talker]: Publishing: 'AAE5303 hello #0'
[INFO] [1769766638.902476060] [env_check_pkg_talker]: Publishing: 'AAE5303 hello #1'
[INFO] [1769766639.402558296] [env_check_pkg_talker]: Publishing: 'AAE5303 hello #2'
[INFO] [1769766639.902864528] [env_check_pkg_talker]: Publishing: 'AAE5303 hello #3'
^C[INFO] [1769766640.148697261] [rclcpp]: signal_handler(SIGINT/SIGTERM)
(aae5303) xinyan@xinyan-virtual-machine:~/aae5303/PolyU-AAE5303-env-smork-test$ ```
ros2 run env_check_pkg listener
> ^C
(aae5303) xinyan@xinyan-virtual-machine:~/aae5303/PolyU-AAE5303-env-smork-test$ ros2 run env_check_pkg listener
[INFO] [1769766713.067831517] [env_check_pkg_listener]: AAE5303 listener awaiting messages.
^C[INFO] [1769766714.862446564] [rclcpp]: signal_handler(SIGINT/SIGTERM)
(aae5303) xinyan@xinyan-virtual-machine:~/aae5303/PolyU-AAE5303-env-smork-test$ ros2 run env_check_pkg listener
[INFO] [1769766723.256127143] [env_check_pkg_listener]: AAE5303 listener awaiting messages.
[INFO] [1769766723.400765286] [env_check_pkg_listener]: I heard: 'AAE5303 hello #3'
[INFO] [1769766723.901319321] [env_check_pkg_listener]: I heard: 'AAE5303 hello #4'
[INFO] [1769766724.399595062] [env_check_pkg_listener]: I heard: 'AAE5303 hello #5'
[INFO] [1769766724.901239030] [env_check_pkg_listener]: I heard: 'AAE5303 hello #6'
[INFO] [1769766725.400200927] [env_check_pkg_listener]: I heard: 'AAE5303 hello #7'
[INFO] [1769766725.900676211] [env_check_pkg_listener]: I heard: 'AAE5303 hello #8'
[INFO] [1769766726.400209893] [env_check_pkg_listener]: I heard: 'AAE5303 hello #9'
[INFO] [1769766726.900994139] [env_check_pkg_listener]: I heard: 'AAE5303 hello #10'
[INFO] [1769766727.401733637] [env_check_pkg_listener]: I heard: 'AAE5303 hello #11'
[INFO] [1769766727.901145446] [env_check_pkg_listener]: I heard: 'AAE5303 hello #12'
[INFO] [1769766728.402393313] [env_check_pkg_listener]: I heard: 'AAE5303 hello #13'
[INFO] [1769766728.900603898] [env_check_pkg_listener]: I heard: 'AAE5303 hello #14'
[INFO] [1769766729.402115518] [env_check_pkg_listener]: I heard: 'AAE5303 hello #15'
[INFO] [1769766729.901977941] [env_check_pkg_listener]: I heard: 'AAE5303 hello #16'
[INFO] [1769766730.401353091] [env_check_pkg_listener]: I heard: 'AAE5303 hello #17'
[INFO] [1769766730.900918638] [env_check_pkg_listener]: I heard: 'AAE5303 hello #18'
[INFO] [1769766731.400579340] [env_check_pkg_listener]: I heard: 'AAE5303 hello #19'
[INFO] [1769766731.901578607] [env_check_pkg_listener]: I heard: 'AAE5303 hello #20'
[INFO] [1769766732.402502428] [env_check_pkg_listener]: I heard: 'AAE5303 hello #21'
[INFO] [1769766732.901761542] [env_check_pkg_listener]: I heard: 'AAE5303 hello #22'
[INFO] [1769766733.400997712] [env_check_pkg_listener]: I heard: 'AAE5303 hello #23'
[INFO] [1769766733.901333329] [env_check_pkg_listener]: I heard: 'AAE5303 hello #24'
[INFO] [1769766734.401239076] [env_check_pkg_listener]: I heard: 'AAE5303 hello #25'
[INFO] [1769766734.900751591] [env_check_pkg_listener]: I heard: 'AAE5303 hello #26'
[INFO] [1769766735.401237917] [env_check_pkg_listener]: I heard: 'AAE5303 hello #27'
[INFO] [1769766735.901298704] [env_check_pkg_listener]: I heard: 'AAE5303 hello #28'
[INFO] [1769766736.402134990] [env_check_pkg_listener]: I heard: 'AAE5303 hello #29'
^C[INFO] [1769766742.634929880] [rclcpp]: signal_handler(SIGINT/SIGTERM)
(aae5303) xinyan@xinyan-virtual-machine:~/aae5303/PolyU-AAE5303-env-smork-test$ ros2 launch env_check_pkg env_check.launch
file 'env_check.launch' was not found in the share directory of package 'env_check_pkg' which is at '/home/xinyan/aae5303/PolyU-AAE5303-env-smork-test/install/env_check_pkg/share/env_check_pkg'
(aae5303) xinyan@xinyan-virtual-machine:~/aae5303/PolyU-AAE
5303-env-smork-test$ ros2 launch env_chec
k_pkg env_check.launch
file 'env_check.launch' was not found in the share directory of package 'env_check_pkg' which is at '/home/xinyan/aae5303/PolyU-AAE5303-env-smork-test/install/env_check_pkg/share/env_check_pkg'
(aae5303) xinyan@xinyan-virtual-machine:~/aae5303/PolyU-AAE5303-env-smork-test$ 



**Why this worked:**  
_[Brief explanation]_
This solution fixed the core ROS 2 environment and package compilation issues by properly building the workspace to generate compiled executable files/launch file installations, loading critical environment variables for terminal recognition of the env_check_pkg package, and configuring standard ROS 2 launch file installation rules in setup.py—ensuring launch files are installed to the official share directory where ros2 launch can detect them. All steps align with ROS 2’s official build and package management protocols, eliminating "package not found" and "launch file not found" errors caused by uncompiled code, unloaded environment variables, and missing launch file configuration.

-

## 6. Reflection (3–5 sentences)

Short but thoughtful:

- What did you learn about configuring robotics environments?
- What surprised you?
- What would you do differently next time (backup, partitioning, reading error logs, asking better AI questions)?
- How confident do you feel about debugging ROS/Python issues now?

**Your reflection:**

launch files need explicit configuration in setup.py/CMakeLists.txt, and the workspace’s setup.bash must be sourced for package recognition. Virtual environment and ROS 2 setup must align, and small details like case-sensitive filenames can cause critical failures.
I was surprised that tiny config oversights (e.g., a missing line in setup.py) could break the whole ROS 2 environment, even with valid core code, and that ROS 2 does not auto-detect non-executable files like general Python projects do.
Next time, I’ll back up config files first, read error logs to pinpoint root causes, ask AI context-rich questions, and compile only target ROS 2 packages for efficient debugging.
I’m still not proficient with ROS 2 and need to practice more.
---

## 7. Declaration

✅ **I confirm that I performed this setup myself and all screenshots/logs reflect my own environment.**

**Name:**  
_[Xinyan Shen]_

**Student ID:**  
_[25072438g]_

**Date:**  
_[Date of submission]_

---

## Submission Checklist

Before submitting, ensure you have:

- [ ] Filled in all system information
- [ ] Included actual terminal outputs (not just screenshots)
- [ ] Provided at least 2 screenshots (Python tests + ROS talker/listener)
- [ ] Documented 2–3 real problems with solutions
- [ ] Completed the AI usage section with exact prompts
- [ ] Written a thoughtful reflection (3–5 sentences)
- [ ] Signed the declaration

---

**End of Report**
