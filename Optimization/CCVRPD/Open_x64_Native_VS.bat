@echo off
REM 1. Project
set PROJECT_PATH=D:\_00_MAFP\_00_github_repositories\Algorithms\Optimization\CCVRPD

REM 2. Call the script to configure VS 2022 for x64 enviroment.
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"

REM 3. Change to dir project.
cd /D "%PROJECT_PATH%"

REM 4. Open VSCode in the current folder
code .