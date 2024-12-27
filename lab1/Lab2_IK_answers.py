import numpy as np
from scipy.spatial.transform import Rotation as R
import copy

def RotFromAxistoAxis(veca,vecb):
    if veca[0] ==  vecb[0] and veca[1] ==  vecb[1] and veca[2] == vecb[2]:
        return  np.array([0,0,0,1])
    if veca[0] == - vecb[0] and veca[1] == - vecb[1] and veca[2] == - vecb[2]:
        return  np.array([veca[0],veca[1],veca[2],0])

    w = 1 + np.dot(veca,vecb)
    xyz = [veca[1] * vecb[2] - vecb[1] * veca[2], veca[2]*vecb[0] - vecb[2] * veca[0],veca[0]*vecb[1] - vecb[0] * veca[1]]

    return np.array([xyz[0],xyz[1],xyz[2],w])

def getHeuristicIkData(metaData,joint_position):
    offsets = np.zeros(len(joint_position) * 3)
    offsets = np.reshape(offsets,[-1,3])
    rotation = np.zeros(len(joint_position) * 3)
    rotation = np.reshape(rotation,[-1,3])

    initial_pos = metaData.joint_initial_position
    joint_parent = metaData.joint_parent

    for joint in range(0,len(joint_position)):
        parId = joint_parent[joint]
        offset = np.zeros(3)
        rot = np.zeros(3)
        if parId == -1:
            offset = initial_pos[joint]
        else:
            offset = initial_pos[joint] - initial_pos[parId]
            joint_offset_new = joint_position[joint] - joint_position[parId]
            rot = RotFromAxistoAxis(offset,joint_offset_new).as_euler("XYZ",degrees= True)
        offsets[joint] = offset
        rotation[joint] = rot    
    return rotation

def getHeuristicIkDataPath(ik_path,joint_position,joint_orientation):
    lenPath = len(ik_path)
    ret_position = np.zeros(lenPath * 3)
    ret_position = ret_position.reshape([-1,3])
    ret_orientation = np.zeros(lenPath * 4)
    ret_orientation = ret_orientation.reshape([-1,4])
    for i in range(0,lenPath):
        ret_position[i] = joint_position[ik_path[i]]
        ret_orientation[i] = joint_orientation[ik_path[i]]
    return ret_position,ret_orientation

def FABRIK_backward(ik_bone_path,path_position,target_pos):
    new_position = copy.deepcopy(path_position)
    new_path_rot = np.zeros(len(ik_bone_path) * 4)
    new_path_rot = new_path_rot.reshape([-1,4])
    new_position[len(ik_bone_path) - 1] = target_pos
    
    end = 0
    for i in range(len(ik_bone_path) - 1,end,-1):
        pos_i = path_position[i]
        pos_i_1 = path_position[i - 1]
        offset = pos_i - pos_i_1
        length = np.linalg.norm(offset)

        move_dir = new_position[i - 1] - new_position[i]
        move_dir = move_dir / np.linalg.norm(move_dir)
        new_position[i - 1] = new_position[i] + move_dir * length
        offset_norm = offset / length
        new_path_rot[i] = RotFromAxistoAxis(offset_norm,-move_dir)
        rot = R.from_quat(new_path_rot[i]).as_euler("XYZ",degrees=True)

        assert(np.isnan(new_position[i][0]) != True and np.isnan(new_position[i][1]) != True and np.isnan(new_position[i][2]) != True)
    new_path_rot[0] = np.array([0,0,0,1])
    new_position[0] = path_position[0]


    return new_position,new_path_rot

def FABRIK_forward(ik_bone_path,new_path_position,path_position):
    new_path_rot = np.zeros(len(ik_bone_path) * 4)
    new_path_rot = np.reshape(new_path_rot,[-1,4])
    new_path_rot[0] = np.array([0,0,0,1])

    end = len(new_path_position)
    for i in range(1,end):
        offset = path_position[i] - path_position[i-1]
        length = np.linalg.norm(offset)

        move_dir = new_path_position[i] - new_path_position[i - 1]
        move_dir = move_dir / np.linalg.norm(move_dir)
        new_path_position[i] = new_path_position[i - 1] + move_dir * length
        new_path_rot[i] = RotFromAxistoAxis(offset/length,move_dir)
        assert(np.isnan(new_path_position[i][0]) != True and np.isnan(new_path_position[i][1]) != True and np.isnan(new_path_position[i][2]) != True)
    
    return new_path_position,new_path_rot

def ApplyFullBofyIK(path2,path,metaData,new_path_rot,new_path_pos,position,orientation):
    b_apply_full_ik = True
    new_position = copy.deepcopy(position)
    new_orientation = orientation
    updated = np.zeros(new_position.shape[0])
    last_rot = R.from_quat(new_orientation[path[-1]]) * R.from_quat(new_orientation[path[-2]]).inv()
    last_rot_e = last_rot.as_euler("XYZ",degrees= True)
    if len(path2) > 1:
        updated[path2[-1]] = 1
        new_position[path2[-1]] = new_path_pos[len(path2) - 1]
        for i in range(0,len(path2) - 1):
            joint = path[i]
            new_orientation[joint] = (R.from_quat(new_orientation[joint]) * R.from_quat(new_path_rot[i])).as_quat()                 
            new_position[joint] = new_path_pos[i]
            updated[joint] = 1
    for i in range(len(path2),len(path)):
        joint = path[i]
        joint_par = metaData.joint_parent[joint]
        new_position[joint] = new_path_pos[i]
        new_orientation[joint_par] = (R.from_quat(new_path_rot[i]) * R.from_quat(new_orientation[joint_par])).as_quat()
        updated[joint] = 1
    new_orientation[path[-1]] = (last_rot *R.from_quat(new_orientation[path[-2]]) ).as_quat()
    if b_apply_full_ik:
        ## 根据initial pos， 计算原来的rotation，应用新的ik信息
        for joint in range(0,len(position)):
            if updated[joint]:
                continue
            joint_par = metaData.joint_parent[joint]
            if joint_par != -1:
                if metaData.joint_name[joint] == "lWrist_end":
                    a = 3
                offset = (position[joint] - position[joint_par]) 
                offsetInitial = metaData.joint_initial_position[joint] - metaData.joint_initial_position[joint_par]
                offset = offset / np.linalg.norm(offset)
                new_position[joint] = new_position[joint_par] + R.from_quat(new_orientation[joint_par]).apply(offsetInitial)
                new_offset = new_position[joint] - new_position[joint_par]
                new_offset = new_offset / np.linalg.norm(new_offset)
                #根据新的位置，计算当前joint的朝向
                rot_new = R.from_quat(RotFromAxistoAxis(offset,new_offset))

                new_orientation[joint] = (rot_new * R.from_quat(new_orientation[joint])).as_quat()
                updated[joint] = 1
            else:
                continue

    return new_position,new_orientation

def FABRIK(meta_data, joint_positions, joint_orientations, target_pose,path,path1,path2):

    step = 0    
    ik_joint_pos = joint_positions
    ik_joint_ori = joint_orientations
    while(True):
        path_position,path_orientation = getHeuristicIkDataPath(path,ik_joint_pos,ik_joint_ori)
        step = step + 1
        new_path_pos,new_path_rot = FABRIK_backward(path,path_position,target_pose)
        #if debug :
        #    return ApplyFullBofyIK(path2,path,meta_data,new_path_rot,new_path_pos,joint_positions,ik_joint_ori,True)
        new_path_pos,new_path_rot = FABRIK_forward(path,new_path_pos,path_position)
        #if debug:
        #    return ApplyFullBofyIK(path2,path,meta_data,new_path_rot,new_path_pos,ik_joint_pos,ik_joint_ori,debug)
        ik_joint_pos,ik_joint_ori = ApplyFullBofyIK(path2,path,meta_data,new_path_rot,new_path_pos,joint_positions,ik_joint_ori)
        delta = np.linalg.norm(target_pose - ik_joint_pos[path[-1]])
        if delta < 0.01 or step == 2 :
            return ik_joint_pos,ik_joint_ori
    return  joint_positions,joint_orientations
        

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
    return FABRIK(meta_data,joint_positions,joint_orientations,target_pose,path,path1,path2)
    

def part2_inverse_kinematics(meta_data, joint_positions, joint_orientations, relative_x, relative_z, target_height):
    """
    输入lWrist相对于RootJoint前进方向的xz偏移，以及目标高度，IK以外的部分与bvh一致
    """
    targetPos = np.array([relative_x,0,relative_z])
    targetPos = targetPos + joint_positions[0]
    targetPos[1] = target_height
    return part1_inverse_kinematics(meta_data,joint_positions,joint_orientations,targetPos)

def bonus_inverse_kinematics(meta_data, joint_positions, joint_orientations, left_target_pose, right_target_pose):
    """
    输入左手和右手的目标位置，固定左脚，完成函数，计算逆运动学
    """
    
    return joint_positions, joint_orientations