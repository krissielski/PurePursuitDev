@ECHO OFF
for %%I in (.) do set AutoDirName=%%~nxI

set CurrPath=%CD%
cd ..\bin
python RobotSim.py %AutoDirName%
cd %CurrPath%



