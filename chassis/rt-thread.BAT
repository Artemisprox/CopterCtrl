SET PATH=C:\Keil_v5\ARM\ARMCC\Bin;C:\ProgramData\Oracle\Java\javapath;C:\Program Files (x86)\Common Files\Oracle\Java\javapath;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;C:\Windows\System32\OpenSSH\;C:\Program Files\Microsoft SQL Server\Client SDK\ODBC\110\Tools\Binn\;C:\Program Files (x86)\Microsoft SQL Server\120\Tools\Binn\;C:\Program Files\Microsoft SQL Server\120\Tools\Binn\;C:\Program Files\Microsoft SQL Server\120\DTS\Binn\;C:\Program Files (x86)\Windows Kits\8.1\Windows Performance Toolkit\;C:\Program Files (x86)\NVIDIA Corporation\PhysX\Common;C:\Program Files\MATLAB\R2019b\runtime\win64;C:\Program Files\MATLAB\R2019b\bin;C:\Program Files (x86)\IVI Foundation\VISA\WinNT\Bin;C:\ProgramData\Anaconda3\Scripts;C:\Users\lenovo\AppData\Local\GitHubDesktop\app-2.5.3\resources\app\git\mingw64\libexec\git-core;C:\Users\lenovo\AppData\Local\GitHubDesktop\app-2.5.3\resources\app\git\mingw64\bin;C:\Users\lenovo\AppData\Local\GitHubDesktop\app-2.5.3\resources\app\git\cmd;C:\Users\lenovo\AppData\Local\Programs\Python\Python38-32\Scripts\;C:\Users\lenovo\AppData\Local\Programs\Python\Python38-32\;C:\Users\lenovo\AppData\Local\Microsoft\WindowsApps;C:\Users\lenovo\AppData\Local\GitHubDesktop\bin;C:\Users\lenovo\AppData\Local\Programs\Microsoft VS Code\bin
SET CPU_TYPE=STM32F427VITx
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=rt-thread
SET CPU_CLOCK=0x00B71B00
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\build\keil\obj\main.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\build\keil\obj\user_uart.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\build\keil\obj\dji_rc.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\build\keil\obj\backtrace.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\build\keil\obj\div0.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\build\keil\obj\showmem.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\build\keil\obj\cpuport.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmAsm" --Via ".\build\keil\obj\context_rvds._ia"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\build\keil\obj\pin.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\build\keil\obj\serial.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\build\keil\obj\completion.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\build\keil\obj\dataqueue.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\build\keil\obj\pipe.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\build\keil\obj\ringblk_buf.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\build\keil\obj\ringbuffer.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\build\keil\obj\waitqueue.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\build\keil\obj\workqueue.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\build\keil\obj\board.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmLink" --Via ".\build\keil\Obj\rt-thread.lnp"
"C:\Keil_v5\ARM\ARMCC\Bin\fromelf.exe" ".\build\keil\Obj\rt-thread.axf" --i32combined --output ".\build\keil\Obj\rt-thread.hex"
fromelf --bin .\build\keil\Obj\rt-thread.axf --output rtthread.bin
