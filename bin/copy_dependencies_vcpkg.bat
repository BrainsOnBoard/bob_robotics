ECHO OFF
SETLOCAL ENABLEDELAYEDEXPANSION

REM Loop through dependencies of dll/executable, filtering out lines which contain dll names
FOR /F %%d IN ('dumpbin /dependents %1 ^| FINDSTR /I /R "[a-zA-Z_-]*\.dll"') DO (
    REM If dll is supplied by VCPKG (others are likely to be in system32)
    SET "VCPKG_FILE=%~2\bin\%%d"
    IF EXIST !VCPKG_FILE! (
        REM If newer file exists in VCPKG, copy
        XCOPY /d /q "!VCPKG_FILE!" "%~dp1" > nul
        
        REM recurse as dll may also have dependencies
        CALL %~dp0copy_dependencies_vcpkg.bat "%~dp1\%%d" "%~2"
    )
)

exit 0