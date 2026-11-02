# WildSonic Firmware

Firmware is designed to support different platforms (e.g. Simulation, Teensy, and MAX78000)

## 0: Board Setup

To use both the MAX78000 FTHR Board with the Mikroe Board, the following connections are needed in addition to external power for both modules.

| MAXIM PIN   | INTERFACE   | MIKROE PIN |
| ----------- | ----------  | ----------- |
| P1_0 (J8 8) | RX <- TX    | TX (14) |
| P1_1 (J8 7) | TX -> RX    | RX (13) |
| P2_6 (J8 14)| RTS -> RTS  | RTS (3) |
| P2_7 (J8 15)| CTS <- CTS  | CTS (15) |
| 3V3 (J8 2)  | VCC -> VCC  | 3V3 (7) |
| GND (J8 4)  | GND <-> GND | GND (8) |
| P1_6 (J4 4) | RST -> RST  | PWK (2) |

Note : The VCC needs to be connected between MAXIM and MIKROE since the voltage level translator on the MIKROE board needs a reference 3V3 voltage to translate the board inputs to 1V8 for UBLOX IC and the UBLOX IC outputs to 3V3 for board output.

## 1. Environment Setup

### 1.1 Local Repository setup
**IMPORTANT**: Make sure to add GitHub SSH Keys to your local system before cloning the repository.\
Guide: [Add SSH key to GitHub](https://docs.github.com/en/github/authenticating-to-github/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)

```
git clone --recurse-submodules git@github.com:wildsonictech/WildSonic_Firmware.git
```
OR
```
git clone git@github.com:wildsonictech/WildSonic_Firmware.git
git submodule update --init --recursive
```

**For specific branch**:
```
git clone --recurse-submodules -b <branch_name> git@github.com:wildsonictech/WildSonic_Firmware.git
```

**Cloning your fork**: Same procedure as above but make sure git path matches your fork, which is of format:
```
git@github.com:<YOUR GITHUB ID>/WildSonic_Firmware.git
```

On Windows, make sure to clone the repo to a directory somewhere below C: drive, not under WSL home directory.
You will use Ubuntu 20.04 WSL2 terminal to build. Then in WSL2 terminal you can navigate to that folder via /mnt/c/.
Setup for WSL2 tools is the same as Linux.

**Pull changes**

As a one-time setup for all repositories, configure git to update submodules on pull:
```
git config --global submodule.recurse true
```

## 1.2 Upstream remote repositories
When cloning a forked repository, it is recommended that an upstream remote repository is added pointing to the original wildsonic repository. This allows the developer to merge changes from the main repository into their local code. This is especially needed for resolving conflicts.

### 1.2.1 Setup wildsonic remote repository

To add **wildsonic** remote repository:
```
git remote add wildsonic git@github.com:wildsonictech/WildSonic_Firmware.git
```

You should now have **origin** and **wildsonic** remote repositories for your cloned code.

### 1.2.2 View remote repositories

To view remote repositories that were set up:
```
git remote -v
```

### 1.2.3 Merge changes that were pushed into wildsonic repository

To pull changes from main branch on **wildsonic** remote repository:
```
git pull wildsonic main
```

### 1.2.4 Pull changes that were pushed into origin repository

To pull changes from branch on **origin** remote repository:
```
git pull origin <branch name>
```

### 1.2.5 Push changes to origin repository

To push changes to branch on **origin** remote repository:
```
git push origin HEAD
```


### 1.3 Compiler setup

#### 1.3.1 Simulation
We use the GNU tools for compile. This comes standard in Linux environment. SIM requires no additional tools.

#### 1.3.2 MAX78000

On Linux, a shortcut to installing ARM GNU tools:
```
sudo apt-get install gcc-arm-none-eabi
```

On Linux, to install RISCV GNU tools:
```
sudo apt install npm
sudo npm install --global xpm@0.10.8
xpm install --global @xpack-dev-tools/riscv-none-embed-gcc@10.1.0-1.1.1 --verbose
```

Add the following to ~/.bashrc
```
# Add riscv gcc path
export PATH="/home/$USER/.local/xPacks/@xpack-dev-tools/riscv-none-embed-gcc/10.1.0-1.1.1/.content/bin:$PATH"
```

Other tools:
```
sudo apt install srecord
sudo apt install libboost-all-dev
sudo apt install libboost-thread-dev
```

For more information, check out the [Install Developer Tools Guide](https://github.com/MaximIntegratedAI/MaximAI_Documentation/blob/master/MAX78000_Evaluation_Kit/README.md#installing-the-developer-tools-linux).
**Make sure to run the Windows installer as administrator**

Also check out:
* [Install RISCV Tools Guide](https://xpack.github.io/riscv-none-embed-gcc/install/)
* [RISCV Debugging Guide](https://github.com/Analog-Devices-MSDK/VSCode-Maxim/wiki/RISC-V-Debugging-Guide)
* [RISCV Debugger Limitations](https://github.com/Analog-Devices-MSDK/VSCode-Maxim#debugger-limitations)
* [PICO board firmware images](https://github.com/MaximIntegratedMicros/max32625pico-firmware-images)
* [Installation and startup instructions for the Maxim SDK with VSCode](https://github.com/Analog-Devices-MSDK/VSCode-Maxim)

<br>

## 2. Build

### 2.1 Clean all files from all builds

To clean:
```
cd build
make clean
```

### 2.2 Compile

To build SIM target (output file should be build/out/SIM/SIM.out):
```
cd build
make TARGET=SIM
```

To build for Maxim target (output file should be build/out/MAX78000/MAX78000-combined.elf):
```
cd build

# Only select one of the builds from below.
# If you need to switch between different FW builds
# or if a header file has been modified,
# make sure to do a clean first.
make clean

# Build FW for WildSonic Board (default latest revision)
make -j TARGET=MAX78000 BOARD=WS_Rev1

# Build FW for WildSonic Board (default latest revision) without GSM (Standalone Recorder only)
make -j TARGET=MAX78000 BOARD=WS_Rev1 COMM_CORE=-1

# Build FW for WildSonic Board older revision (e.g. 1.2)
make -j TARGET=MAX78000 BOARD=WS_Rev1 BOARD_REVISION=2

# Build FW for WildSonic Board older revision (e.g. 1.4)
make -j TARGET=MAX78000 BOARD=WS_Rev1 BOARD_REVISION=4

# Build FW for WildSonic Board (default latest revision) with older Power Card (< 2.2)
make -j TARGET=MAX78000 BOARD=WS_Rev1 RTC_PERIPHERAL=none

# Build FW for WildSonic Board older revision (e.g. 1.2) with older Power Card (< 2.2)
make -j TARGET=MAX78000 BOARD=WS_Rev1 BOARD_REVISION=2 RTC_PERIPHERAL=none

# Build FW for WildSonic Board older revision (e.g. 1.4) with older Power Card (< 2.2)
make -j TARGET=MAX78000 BOARD=WS_Rev1 BOARD_REVISION=4 RTC_PERIPHERAL=none

# Build FW for MAX78000 FTHR Board (default uses codec max9867)
make -j TARGET=MAX78000 BOARD=FTHR_RevA

# Build FW for MAX78000 FTHR Board using on-board I2S microphone
make -j TARGET=MAX78000 BOARD=FTHR_RevA I2S_PERIPHERAL=sph0645

# Build FW for WildSonic Board with BLE CORE CUSTOM CC2541 FW Support
make -j TARGET=MAX78000 COMM_CORE=-1 BLE_FW=BLE_CORE_CUSTOM

# Build FW for WildSonic Board without BLE CORE
make -j TARGET=MAX78000 COMM_CORE=-1 BLE_CORE=-1
```

On WSL2, since you checked out the project folder somewhere in C: drive. You can navigate to that folder via the mounted /mnt/c/ drive in the WSL2 terminal.

### 2.3 Lint

Code format is enforced by clang-format. To check if code is correct:
1. Install docker
2. Build and run docker with clang formats
```
sudo docker build -t clang-format-lint github.com/DoozyX/clang-format-lint-action
sudo docker run -it --rm --workdir /src -v $(pwd):/src clang-format-lint --clang-format-executable /clang-format/clang-format12 -r --exclude .git .
```

Alternatively you can install the VS code plugin to format your code.

On **Ubuntu**:
Make sure to install clang-format:
```
sudo apt install clang-format-12
```

Then add the following lines to your .vscode/settings.json:
```
"clang-format.executable": "/usr/bin/clang-format-12",
"editor.formatOnSave": true
```

On **Windows**:
[Download and install LLVM-12.0.1](https://github.com/llvm/llvm-project/releases/tag/llvmorg-12.0.1)

Note: You will need to add the path to the binaries (e.g. *C:\Program Files\LLVM\bin*) to the System Path environment
variable.

Then add the following lines to your .vscode/settings.json:
```
"clang-format.executable": "clang-format",
"editor.formatOnSave": true
```

Note: Other plugins may conflict with this one, so make sure to uninstall other extensions that are not mentioned in this README to get the clang-format working in VS code.

### 2.4 Spellcheck

Spelling in in-code comments and variable names are checked in the workflow.

To install:
```
sudo apt-get install aspell=0.60.8-1ubuntu0.1
pip3 install pyspelling==2.7.2
```

To run:
```
pyspelling -v
```

If a word is falsely identified as an error, then an exception to spellcheck needs to be added.
The word can be added to build/pyspelling-en-custom.txt


## 3. Editor/IDE Setup

### 3.1 VS Code

Project files are already created. Just open the project by navigating to the project directory and typing:
```
code .
```

#### 3.1.1 VS Code plugins that need to be installed
1. C/C++ - Microsoft
2. Clang-Format - xaver
3. Code Spell Checker - Street Side Software
4. Docker - Microsoft
5. docs-markdown - Microsoft
6. Doxygen Document Generator - Christoph Schlosser
7. PlantUML - jebbs (requires [java runtime environment](https://www.java.com/download) to be installed)
8. Cortex-Debug Extension

#### 3.1.2 Running SIM

You need to build the binary first. But then hit the play button in VS code to start the SIM debug with gdb.

#### 3.1.3 Running Firmware

Run the Maxim SDK installer executable, and ensure that "Visual Studio Code Support" is enabled for your installation. And
You need to build the binary first. And then go to Terminal > Run flash task. or the shortcut Ctrl+Shift+B.
Here are the tasks to run (configured by the .vscode/tasks.json file):

**Flash**
* Launching this task automatically runs the Build task first. Then, it flashes the output binary to the microcontroller.
* It uses the GDB load and compare-sections commands, and handles launching an OpenOCD internally via a pipe connection.
* The flashed program will be halted until the microcontroller is reset, power cycled, or a debugger is connected.
* A debugger must be connected correctly to use this task. Refer to the datasheet of your microcontroller's evaluation board for instructions.

**Flash & Run**
* This is the same as the Flash task, but it also will launch execution of the program once flashing is complete.

**Erase Flash**
* Completely erases all of the application code in the flash memory bank.
* Once complete, the target microcontroller will be effectively "blank".
* This can be useful for recovering from Low-Power (LP) lockouts, bad firmware, etc.

### 3.1.4 Debugging
Debugging is enabled by Visual Studio Code's integrated debugger. Launch configurations can be found in the .vscode/launch.json file.

*  Note: Flashing does not happen automatically when launching the debugger. Run the "Flash" build task for your program before debugging.

**Launching the Debugger**
Attach your debugger to the SWD port on the target microcontroller.

1. Flash the program to the microcontroller with the "Flash" Build Task. Flashing does not happen automatically when launching the debugger.
2. Launch the debugger with Run > Start Debugging, with the shortcut F5, or via the Run and Debug window (Ctrl + Shift + D) and the green "launch" arrow.
3. The debugger will launch a GDB client & OpenOCD server, reset the microcontroller, and should break on entry into main if used GDB (ARM M4) debugging mode and if GDB_NO_RESET(ARM M4) used it won't reset the board
4. Use GDB(RISCV) to debug the RISC V core and it won't reset the system.

For more information look into the below git repository:
[Analog devices-MAXIM-SDK](https://github.com/Analog-Devices-MSDK/VSCode-Maxim)

### 3.2 Eclipse

To be used on Windows via Maxim SDK Software. Make sure to open the **Eclipse MaximSDK** application instead of the standard Eclipse. Otherwise, the environment variables for debug will not be set up properly.

Since the project and launch files are already created, just include the project in your workspace via **File->Open Projects from File System...**

Build is not supported on Windows and Eclipse is only used for launching the debug. Use the WSL2 terminal to compile.

#### 3.2.1 Running on Max78000 Fthr Board

Just hit the debug button for target **WLF** in Eclipse MaximSDK. By default, this will load the executable into the Flash and jump to init. In this view, only the ARM core can be stepped.

After programming the executable, the device will boot on it's own on the next power-up with the programmed executable file.

If the desire is to just break into the ARM core without programming or jumping to init, then under **Debug Configurations** for **WLF**, uncheck **Initial Reset** and **Load Executable** before hitting the debug button.

At this point, RISC-V debugging is also available. Make sure to connect the debugger to the swd port on the board. Also copy the max78000_riscv.cfg file (Download from the RISCV Debugging Guide or [Download Here](https://github.com/Analog-Devices-MSDK/VSCode-Maxim/files/9587266/max78000_riscv.zip)) to your MaximSDK/Tools/OpenOCD/scripts/target directory. Hit the debug button for target **WLF RISCV** in Eclipse MaximSDK and the core is now visible for debug.

Note: The binary must be compiled with **ENABLE_RISCV_DEBUG** set to 1. I2S must be disabled as it shares the same pins as the JTAG.

Also check out:
* [RISCV Debugging Guide](https://github.com/Analog-Devices-MSDK/VSCode-Maxim/wiki/RISC-V-Debugging-Guide)
* [RISCV Debugger Limitations](https://github.com/Analog-Devices-MSDK/VSCode-Maxim#debugger-limitations)

## 4. Trace Log Debugging

The trace log can be useful for debugging long runs where the UART console is not connected or is unresponsive. Nearly all prints to the UART console are added to the trace log. The trace log is persistent across power cycle, which makes it very useful when deployed in field.

### 4.1 Run
The trace log is saved in binary format (e.g. DEVXXXXXXX_log.bin) in the top-level directory of the sd card. You will need to parse the binary using the trace parser in tools/trace_parse.

Install dependencies:
```
sudo apt install libboost-all-dev
```

First compile the trace parser:
```
make
cd out/
```

To run the parser:
```
./trace_parse DEVXXXXXXX_log.bin > trace_dump.log
```

### 4.2 Interpretation of Log

The Log entry will be of the format:
```
<GMT DATETIME> C:<CORE> [F:(<FILE_NUMBER>), L:<LINE_NUMBER>] WITH ARG DATA (<BYTES> bytes):
HEX: <ARG1 IN HEX FORMAT>
ASCII: <ARG1 IN ASCII FORMAT>
HEX: <ARG2 IN HEX FORMAT>
ASCII: <ARG2 IN ASCII FORMAT>
...
```

The corresponding file name can be obtained by looking up the FILE_NUMBER in the TraceFile enum in lib/trace/include/trace.hpp. With the file name and LINE_NUMBER, the point where the log occurred can be seen. The ARG DATA in the log corresponds to the context arguments provided to the LOG_MSG_WITH_ARGS function. These are the same arguments that are used for console prints.