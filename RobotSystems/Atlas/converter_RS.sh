COMMAND0="shopt -s extglob"
# Change name of .py
COMMAND1="python v2aConverter_RS.py"
COMMAND2="rm *_a.cpp *_b.cpp *_a.hpp *_b.hpp *_a.h *_b.h *_a.txt *_b.txt *_a.urdf *_b.urdf"
COMMAND3="rm -v !(*_c.cpp|*_c.hpp|*_c.h|*_c.txt|*.sh|*.py|*_c.urdf)"
# Change name of current files
COMMAND4="mv Atlas_Definition_c.h Atlas_Definition.h"
COMMAND5="mv Atlas_Dyn_Model_c.hpp Atlas_Dyn_Model.hpp"
COMMAND6="mv Atlas_Dyn_Model_c.cpp Atlas_Dyn_Model.cpp"
COMMAND7="mv Atlas_Kin_Model_c.hpp Atlas_Kin_Model.hpp"
COMMAND8="mv Atlas_Kin_Model_c.cpp Atlas_Kin_Model.cpp"
COMMAND9="mv Atlas_Model_c.hpp Atlas_Model.hpp"
COMMAND10="mv Atlas_Model_c.cpp Atlas_Model.cpp"
COMMAND11="mv CMakeLists_c.txt CMakeLists.txt"
COMMAND12="mv atlas_v3_no_head_c.urdf atlas_v3_no_head.urdf"
#COMMAND13="shopt -u extglob"
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
#$COMMAND13
