@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0"
if exist "%~dp0.venv\Scripts\python.exe" (
  "%~dp0.venv\Scripts\python.exe" -c "import sys; sys.exit(sys.version_info < (3,10))" >nul 2>nul
  if not errorlevel 1 (
    "%~dp0.venv\Scripts\python.exe" run.py %*
    goto done
  )
)
where py >nul 2>nul
if not errorlevel 1 (
  py -3 -c "import sys; sys.exit(sys.version_info < (3,10))" >nul 2>nul
  if not errorlevel 1 (
    py -3 run.py %*
    goto done
  )
)
where python >nul 2>nul
if not errorlevel 1 (
  python -c "import sys; sys.exit(sys.version_info < (3,10))" >nul 2>nul
  if not errorlevel 1 (
    python run.py %*
    goto done
  )
)
set "LUMBAR_PYTHON=%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
if exist "%LUMBAR_PYTHON%" (
  "%LUMBAR_PYTHON%" -c "import sys; sys.exit(sys.version_info < (3,10))" >nul 2>nul
  if not errorlevel 1 (
    "%LUMBAR_PYTHON%" run.py %*
    goto done
  )
)
echo Python 3.10 or newer is required. Install Python and run this file again.
:done
if errorlevel 1 pause
endlocal
