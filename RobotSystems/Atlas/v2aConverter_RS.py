### RobotSystems file converter ###

def main():

# file_path = os.getcwd() + "/RobotSystemConverted/Atlas/"
# read files
    # data_cmd = \
            # np.genfromtxt(file_path+'command.txt', delimiter=None, dtype=(float))
    # data_filter_cmd = \
            # np.genfromtxt(file_path+'filtered_cmd.txt', delimiter=None, dtype=(float))
    # data_torque = \
            # np.genfromtxt(file_path+'torque.txt', delimiter=None, dtype=(float))

# Atlas_Definition.h
    with open("Atlas_Definition.h", "rt") as fin:
        with open("Atlas_Definition_a.h", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("Atlas_Definition_a.h", "rt") as fin:
        with open("Atlas_Definition_b.h", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("Atlas_Definition_b.h", "rt") as fin:
        with open("Atlas_Definition_c.h", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# Atlas_Model.hpp
    with open("Atlas_Model.hpp", "rt") as fin:
        with open("Atlas_Model_a.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("Atlas_Model_a.hpp", "rt") as fin:
        with open("Atlas_Model_b.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("Atlas_Model_b.hpp", "rt") as fin:
        with open("Atlas_Model_c.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# Atlas_Model.cpp
    with open("Atlas_Model.cpp", "rt") as fin:
        with open("Atlas_Model_a.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("Atlas_Model_a.cpp", "rt") as fin:
        with open("Atlas_Model_b.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("Atlas_Model_b.cpp", "rt") as fin:
        with open("Atlas_Model_c.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# Atlas_Kin_Model.hpp
    with open("Atlas_Kin_Model.hpp", "rt") as fin:
        with open("Atlas_Kin_Model_a.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("Atlas_Kin_Model_a.hpp", "rt") as fin:
        with open("Atlas_Kin_Model_b.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("Atlas_Kin_Model_b.hpp", "rt") as fin:
        with open("Atlas_Kin_Model_c.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# Atlas_Kin_Model.cpp
    with open("Atlas_Kin_Model.cpp", "rt") as fin:
        with open("Atlas_Kin_Model_a.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("Atlas_Kin_Model_a.cpp", "rt") as fin:
        with open("Atlas_Kin_Model_b.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("Atlas_Kin_Model_b.cpp", "rt") as fin:
        with open("Atlas_Kin_Model_c.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# Atlas_Dyn_Model.hpp
    with open("Atlas_Dyn_Model.hpp", "rt") as fin:
        with open("Atlas_Dyn_Model_a.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("Atlas_Dyn_Model_a.hpp", "rt") as fin:
        with open("Atlas_Dyn_Model_b.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("Atlas_Dyn_Model_b.hpp", "rt") as fin:
        with open("Atlas_Dyn_Model_c.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# Atlas_Dyn_Model.cpp
    with open("Atlas_Dyn_Model.cpp", "rt") as fin:
        with open("Atlas_Dyn_Model_a.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("Atlas_Dyn_Model_a.cpp", "rt") as fin:
        with open("Atlas_Dyn_Model_b.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("Atlas_Dyn_Model_b.cpp", "rt") as fin:
        with open("Atlas_Dyn_Model_c.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# CMakeLists.txt
    with open("CMakeLists.txt", "rt") as fin:
        with open("CMakeLists_a.txt", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("CMakeLists_a.txt", "rt") as fin:
        with open("CMakeLists_b.txt", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("CMakeLists_b.txt", "rt") as fin:
        with open("CMakeLists_c.txt", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))
 
# atlas_v3_no_head.urdf
    with open("atlas_v3_no_head.urdf", "rt") as fin:
        with open("atlas_v3_no_head_a.urdf", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("atlas_v3_no_head_a.urdf", "rt") as fin:
        with open("atlas_v3_no_head_b.urdf", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("atlas_v3_no_head_b.urdf", "rt") as fin:
        with open("atlas_v3_no_head_c.urdf", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))
 
 # 1. urdf, cmakelist.tx erased redownload
 # 2. figure out why in this file
main()
