@ECHO OFF
for %%I in (.) do set AutoDirName=%%~nxI

set CurrPath=%CD%
cd ..\bin
python PathGen.py %AutoDirName%
cd %CurrPath%

copy output.csv %AutoDirName%_pp.csv



