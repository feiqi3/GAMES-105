import numpy as np
from scipy.spatial.transform import Rotation as R
import copy

def RotFromAxistoAxis(veca,vecb):
    if veca[0] ==  vecb[0] and veca[1] ==  vecb[1] and veca[2] == vecb[2]:
        return  R.from_quat([0,0,0,1])
    if veca[0] == - vecb[0] and veca[1] == - vecb[1] and veca[2] == - vecb[2]:
        return  R.from_quat([veca[0],veca[1],veca[2],0])

    w = 1 + np.dot(veca,vecb)
    xyz = [veca[1] * vecb[2] - vecb[1] * veca[2], veca[2]*vecb[0] - vecb[2] * veca[0],veca[0]*vecb[1] - vecb[0] * veca[1]]

    return np.array([xyz[0],xyz[1],xyz[2],w])

def FABRIK_backward(ik_bone_path,path_position,target_pos):
    new_tar_pos = target_pos
    new_position = copy.deepcopy(path_position)
    new_position[len(ik_bone_path) - 1] = target_pos
    for i in range(len(ik_bone_path) - 2,-1,-1):
        length =  np.linalg.norm( path_position[i + 1] - path_position[i])
        new_vec_dir = (path_position[i + 1] - new_position[i + 1])
        new_vec_dir = new_vec_dir / np.linalg.norm(new_vec_dir)
        new_vec = new_vec_dir * length + new_tar_pos
        new_position[i] = new_vec 
        new_tar_pos = new_vec
    return new_position

def FABRIK_forward(ik_bone_path,new_path_position,begin_pos):
    old_pos = begin_pos
    new_orientation = np.zeros(len(ik_bone_path) * 4)
    np.reshape(new_orientation,[-1,4])
    new_position = copy.deepcopy(new_path_position)
    new_position[0] = begin_pos
    for i in range(0,len(ik_bone_path) - 1):
        vec = new_path_position[i] - new_position[i]
        dir_len = np.linalg.norm(new_path_position[i] - new_path_position[i + 1])
        dir_vec = vec / np.linalg.norm(vec)
        new_position[i + 1] =  new_position[i] + dir_vec * dir_len
        new_orientation[i] = RotFromAxistoAxis([0,1,0],dir_vec)
    new_orientation[len(ik_bone_path) - 1] = R.from_quat([0,0,0,1])
    return new_position,new_orientation

def FABRIK():
    return 

def part1_inverse_kinematics(meta_data, joint_positions, joint_orientations, target_pose):
    """
    完成函数，计算逆运动学
    输入: 
        meta_data: 为了方便，将一些固定信息进行了打包，见上面的meta_data类
        joint_positions: 当前的关节位置，是一个numpy数组，shape为(M, 3)，M为关节数
        joint_orientations: 当前的关节朝向，是一个numpy数组，shape为(M, 4)，M为关节数
        target_pose: 目标位置，是一个numpy数组，shape为(3,)
    输出:
        经过IK后的姿态
        joint_positions: 计算得到的关节位置，是一个numpy数组，shape为(M, 3)，M为关节数
        joint_orientations: 计算得到的关节朝向，是一个numpy数组，shape为(M, 4)，M为关节数
    """
    path, path_name, path1, path2 = meta_data.get_path_from_root_to_end()
    print(path1)
    print(path2)
    print(path_name)
    return 

def part2_inverse_kinematics(meta_data, joint_positions, joint_orientations, relative_x, relative_z, target_height):
    """
    输入lWrist相对于RootJoint前进方向的xz偏移，以及目标高度，IK以外的部分与bvh一致
    """
    
    return joint_positions, joint_orientations

def bonus_inverse_kinematics(meta_data, joint_positions, joint_orientations, left_target_pose, right_target_pose):
    """
    输入左手和右手的目标位置，固定左脚，完成函数，计算逆运动学
    """
    
    return joint_positions, joint_orientations