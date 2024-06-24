from enum import Enum


class PTPMode(Enum):
    """
    0. JUMP_XYZ, ]
        Jump mode,
        (x,y,z,r)
        is the target point in Cartesian coordinate system

    1. MOVJ_XYZ,
        Joint movement,
        (x,y,z,r)
        is the target point in Cartesian coordinate system

    2. MOVL_XYZ,
        Linear movement,
        (x,y,z,r)
        is the target point in Cartesian coordinate system

    3. JUMP_ANGLE,
        Jump mode, (x,y,z,r)
        is the target point in Jointcoordinate system

    4. MOVJ_ANGLE,
        Joint movement,
        (x,y,z,r)
        is the target point in Joint coordinate system

    5. MOVL_ANGLE,
        Linear movement,
        (x,y,z,r)
        is the target point in Joint coordinate system

    6. MOVJ_INC,
        Joint movement increment mode,
        (x,y,z,r)
        is the angle increment in Joint coordinate system

    7. MOVL_INC,
        Linear movement increment mode,
        (x,y,z,r)
        is the Cartesian coordinate increment in Joint coordinate system

    8. MOVJ_XYZ_INC,
        Joint movement increment mode,
        (x,y,z,r)
        is the Cartesian coordinate increment in Cartesian coordinate system

    9. JUMP_MOVL_XYZ,
        Jump movement,
        (x,y,z,r)
        is the Cartesian coordinate increment in Cartesian coordinate system
    """
    JUMP_XYZ = 0
    MOVJ_XYZ = 1
    MOVL_XYZ = 2
    JUMP_ANGLE = 3
    MOVJ_ANGLE = 4
    MOVL_ANGLE = 5
    MOVJ_INC = 6
    MOVL_INC = 7
    MOVJ_XYZ_INC = 8
    JUMP_MOVL_XYZ = 9
