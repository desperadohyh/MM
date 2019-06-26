@echo off
set MATLAB=C:\PROGRA~1\MATLAB\R2017b
set MATLAB_ARCH=win64
set MATLAB_BIN="C:\Program Files\MATLAB\R2017b\bin"
set ENTRYPOINT=mexFunction
set OUTDIR=.\
set LIB_NAME=testMEX_mex
set MEX_NAME=testMEX_mex
set MEX_EXT=.mexw64
call "C:\PROGRA~1\MATLAB\R2017b\sys\lcc64\lcc64\mex\lcc64opts.bat"
echo # Make settings for testMEX > testMEX_mex.mki
echo COMPILER=%COMPILER%>> testMEX_mex.mki
echo COMPFLAGS=%COMPFLAGS%>> testMEX_mex.mki
echo OPTIMFLAGS=%OPTIMFLAGS%>> testMEX_mex.mki
echo DEBUGFLAGS=%DEBUGFLAGS%>> testMEX_mex.mki
echo LINKER=%LINKER%>> testMEX_mex.mki
echo LINKFLAGS=%LINKFLAGS%>> testMEX_mex.mki
echo LINKOPTIMFLAGS=%LINKOPTIMFLAGS%>> testMEX_mex.mki
echo LINKDEBUGFLAGS=%LINKDEBUGFLAGS%>> testMEX_mex.mki
echo MATLAB_ARCH=%MATLAB_ARCH%>> testMEX_mex.mki
echo OMPFLAGS= >> testMEX_mex.mki
echo OMPLINKFLAGS= >> testMEX_mex.mki
echo EMC_COMPILER=lcc64>> testMEX_mex.mki
echo EMC_CONFIG=optim>> testMEX_mex.mki
"C:\Program Files\MATLAB\R2017b\bin\win64\gmake" -B -f testMEX_mex.mk
