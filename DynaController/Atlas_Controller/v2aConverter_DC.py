### RobotSystems file converter ###

def main():
# Atlas_DynaControl_Definition.h
    with open("Atlas_DynaControl_Definition.h", "rt") as fin:
        with open("Atlas_DynaControl_Definition_a.h", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("Atlas_DynaControl_Definition_a.h", "rt") as fin:
        with open("Atlas_DynaControl_Definition_b.h", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("Atlas_DynaControl_Definition_b.h", "rt") as fin:
        with open("Atlas_DynaControl_Definition_c.h", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# Atlas_InvKinematics.cpp
    with open("Atlas_InvKinematics.cpp", "rt") as fin:
        with open("Atlas_InvKinematics_a.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("Atlas_InvKinematics_a.cpp", "rt") as fin:
        with open("Atlas_InvKinematics_b.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("Atlas_InvKinematics_b.cpp", "rt") as fin:
        with open("Atlas_InvKinematics_c.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# Atlas_InvKinematics.hpp
    with open("Atlas_InvKinematics.hpp", "rt") as fin:
        with open("Atlas_InvKinematics_a.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("Atlas_InvKinematics_a.hpp", "rt") as fin:
        with open("Atlas_InvKinematics_b.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("Atlas_InvKinematics_b.hpp", "rt") as fin:
        with open("Atlas_InvKinematics_c.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# Atlas_StateEstimator.cpp
    with open("Atlas_StateEstimator.cpp", "rt") as fin:
        with open("Atlas_StateEstimator_a.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("Atlas_StateEstimator_a.cpp", "rt") as fin:
        with open("Atlas_StateEstimator_b.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("Atlas_StateEstimator_b.cpp", "rt") as fin:
        with open("Atlas_StateEstimator_c.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# Atlas_StateEstimator.hpp
    with open("Atlas_StateEstimator.hpp", "rt") as fin:
        with open("Atlas_StateEstimator_a.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("Atlas_StateEstimator_a.hpp", "rt") as fin:
        with open("Atlas_StateEstimator_b.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("Atlas_StateEstimator_b.hpp", "rt") as fin:
        with open("Atlas_StateEstimator_c.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# Atlas_StateProvider.cpp
    with open("Atlas_StateProvider.cpp", "rt") as fin:
        with open("Atlas_StateProvider_a.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("Atlas_StateProvider_a.cpp", "rt") as fin:
        with open("Atlas_StateProvider_b.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("Atlas_StateProvider_b.cpp", "rt") as fin:
        with open("Atlas_StateProvider_c.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# Atlas_StateProvider.hpp
    with open("Atlas_StateProvider.hpp", "rt") as fin:
        with open("Atlas_StateProvider_a.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("Atlas_StateProvider_a.hpp", "rt") as fin:
        with open("Atlas_StateProvider_b.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("Atlas_StateProvider_b.hpp", "rt") as fin:
        with open("Atlas_StateProvider_c.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# Atlas_interface.cpp
    with open("Atlas_interface.cpp", "rt") as fin:
        with open("Atlas_interface_a.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("Atlas_interface_a.cpp", "rt") as fin:
        with open("Atlas_interface_b.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("Atlas_interface_b.cpp", "rt") as fin:
        with open("Atlas_interface_c.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('valkyrie','atlas'))

# Atlas_interface.hpp
    with open("Atlas_interface.hpp", "rt") as fin:
        with open("Atlas_interface_a.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('VALKYRIE','ATLAS'))
    with open("Atlas_interface_a.hpp", "rt") as fin:
        with open("Atlas_interface_b.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Valkyrie','Atlas'))
    with open("Atlas_interface_b.hpp", "rt") as fin:
        with open("Atlas_interface_c.hpp", "wt") as fout:
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
 
### TEMPLATE
## atlas_v3_no_head.urdf
    # with open("", "rt") as fin:
        # with open("", "wt") as fout:
            # for line in fin:
                # fout.write(line.replace('VALKYRIE','ATLAS'))
    # with open("", "rt") as fin:
        # with open("", "wt") as fout:
            # for line in fin:
                # fout.write(line.replace('Valkyrie','Atlas'))
    # with open("", "rt") as fin:
        # with open("", "wt") as fout:
            # for line in fin:
                # fout.write(line.replace('valkyrie','atlas'))
main()
