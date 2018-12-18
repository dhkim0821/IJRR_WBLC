COMMAND0="shopt -s extglob"
# Change name of .py
COMMAND1="python v2aConverter_DC.py"
COMMAND2="rm *_a.cpp *_b.cpp *_a.hpp *_b.hpp *_a.h *_b.h *_a.txt *_b.txt *_a.urdf *_b.urdf"
COMMAND3="rm -v !(*_c.cpp|*_c.hpp|*_c.h|*_c.txt|*.sh|*.py|*_c.urdf)"
# Change name of current files
COMMAND4="mv Atlas_DynaControl_Definition_c.h Atlas_DynaControl_Definition.h"
COMMAND5="mv Atlas_InvKinematics_c.cpp Atlas_InvKinematics.cpp"
COMMAND6="mv Atlas_InvKinematics_c.hpp Atlas_InvKinematics.hpp"
COMMAND7="mv Atlas_StateEstimator_c.cpp Atlas_StateEstimator.cpp"
COMMAND8="mv Atlas_StateEstimator_c.hpp Atlas_StateEstimator.hpp"
COMMAND9="mv Atlas_StateProvider_c.cpp Atlas_StateProvider.cpp"
COMMAND10="mv Atlas_StateProvider_c.hpp Atlas_StateProvider.hpp"
COMMAND11="mv Atlas_interface_c.cpp Atlas_interface.cpp"
COMMAND12="mv Atlas_interface_c.hpp Atlas_interface.hpp"
COMMAND13="mv CMakeLists_c.txt CMakeLists.txt"

$COMMAND0
echo "converting.. valkyrie->atlas"
$COMMAND1
echo "removing _a, _b files"
$COMMAND2
echo "removing non _c files"
$COMMAND3
echo "convert to orginal format"
$COMMAND4
$COMMAND5
$COMMAND6
$COMMAND7
$COMMAND8
$COMMAND9
$COMMAND10
$COMMAND11
$COMMAND12
$COMMAND13
