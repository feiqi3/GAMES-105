import numpy as np
from scipy.spatial.transform import Rotation as R
import copy

def calc_joint_jacobian(joint_path,joint_path_rot,joint_path_pos,joint_path_orientation):
    """ 
    end_pos: target position of end joi
    joint_path: index of joint in the ik path
    joint_path_rot:euler rotation of joint in its parent space
    joint_path_pos: position of joint in ik path
    joint_path_orientation:quat orientation of joint in ik path
    """
    jacobian = []
    idx = 0
    end_pos = joint_path_pos[len(joint_path) - 1]
    ori_par = R.from_quat([0,0,0,1])
    for joint_idx in range(0,len(joint_path)):
        joint_pos = joint_path_pos[joint_idx]
        joint_rot = joint_path_rot[joint_idx]
        if joint_idx != 0:
            ori_par = R.from_quat(joint_path_orientation[joint_idx - 1])
        r = end_pos - joint_pos
        ax = ori_par.apply(np.array([1,0,0]))
        ay = (ori_par * R.from_euler("XYZ",[joint_rot[0],0,0])).apply(np.array([0,1,0]))
        az = (ori_par * R.from_euler("XYZ",[joint_rot[0],0,0]) * R.from_euler("XYZ",[0,joint_rot[1],0])).apply(np.array([0,1,0]))
        jacobian.append(np.cross(ax,r))
        jacobian.append(np.cross(ay,r))
        jacobian.append(np.cross(az,r))
    return np.concatenate(jacobian).transpose()

def ik_pre_data(ik_path, joint_positions, joint_orientations):
    """
    输出: 
        joint_path_rot:euler rotation of joint in its parent space
        joint_path_pos: position of joint in ik path
        joint_path_orientation:quat orientation of joint in ik path
    """
    joint_path_rot = []
    joint_path_pos = []
    joint_path_orientation = []
    for idx in range(len(ik_path)):
        rot = None
        joint_idx = ik_path[idx]
        if idx == 0:
            rot = R.from_quat(joint_orientations[idx]).as_euler("XYZ",degrees=True)
        else:
            rot = R.from_quat(joint_orientations[idx - 1]).inv() * R.from_quat(joint_orientations[idx])
        joint_path_rot.append(rot.as_euler("XYZ",degrees=True))
        joint_path_pos.append(joint_positions[joint_idx])    
        joint_path_orientation.append(joint_orientations[joint_idx])
    return joint_path_rot,joint_path_pos,joint_path_orientation

def do_ik_jacobian(path,joint_positions,joint_orientations,target_pose,alpha):
    ## so we get a jacobian matrix tanspose in shape 3n x 3
    ## then theta_i+1 = theta_i * alpha * j^T * delta --> 3n  to  3 x n is shape of theta
    ## delta = target_pose - cur_end_pos
    joint_path_rot,joint_path_pos,joint_path_orientation = ik_pre_data(path,joint_positions,joint_orientations)
    jacobian = calc_joint_jacobian(path,joint_path_rot,joint_path_pos,joint_path_orientation)
    factor = alpha * np.dot(jacobian , (target_pose - joint_path_pos[-1]))
    np.reshape(factor,[len(joint_path_orientation),3])
    joint_path_rot = joint_path_rot - factor
    return joint_path_rot

def populateIk(joint_positons,joint_parents,joint_orientations,ik_path_rot,ik_path,ik_path_1,ik_path_2):
    updated = np.zeros(len(joint_positons))
    ## calculate rotation from joint_orientation
    ## calculate joint offset from joint_positoin
    new_joint_orientation = copy.deepcopy(joint_orientations)
    ## ik pass
    if len(ik_path_2) > 1:
        for idx in range(0,len(ik_path_2) - 1):
            joint_idx = ik_path_2[idx]
            ## Q_{I+1} = Q_{i}*R_{i+1} --> Q_{i} = Q_{i+1} * R_{i+1}^T 
            rot_i_add_1 = ik_path_rot[joint_idx]

            rot_i_add_1 = R.from_euler("XYZ",ik_path_rot[joint_idx],degrees=True)
            ori_i = new_joint_orientation[ik_path_2[idx + 1]]*rot_i_add_1.inv()
            new_joint_orientation[ik_path_2[idx + 1]] = ori_i
            updated[joint_idx] = 1
    if len(ik_path_1) > 0:
        for i in range(0,len(ik_path_1) - 1):
            idx_inv =  len(ik_path_1) - 1 - i  
            joint_idx = ik_path_1[i]
            par_rot = None
            par_idx= joint_parents[joint_idx]
            if par_idx == -1:
                par_rot = R.from_quat([0,0,0,1])
            else:
                par_rot = R.from_euler("XYZ",new_joint_orientation[par_idx],degrees=True)
            new_joint_orientation[joint_idx] = par_rot * ik_path_rot[idx_inv]
            updated[joint_idx] = 1
    
## TODO: fast path --> if ik donot change rootJoint --> then one ik pass can do all.

    ## Completely fk pass
    joint_position_new = copy.deepcopy(joint_positons)
    for idx in range(0,len(joint_positons)):
        par_idx = joint_parents[idx]
        par_ori = None
        par_pos = None
        joint_offset = None
        joint_offset = joint_positons[idx]
        if par_idx == -1:
            par_ori = R.from_quat([0,0,0,1])
            par_pos = np.array([0,0,0])
        else :
            par_ori = R.from_euler("XYZ",new_joint_orientation[par_idx],degrees=True)
            joint_offset = joint_offset - joint_positons[par_idx]
            par_pos = joint_positons[par_idx]
        joint_position_new[idx] = par_pos + par_ori.apply(joint_offset)
        if updated[idx] == 0 : 
            ## Q_pi * rotation_i =  Qi ---> rotation_i =inv(Q_pi) * Qi 
            ## Q_i_new = Q_pi_new * rotation_i
            new_joint_par_ori = None
            if new_joint_orientation[par_idx] == -1:
                new_joint_par_ori = R.from_quat([0,0,0,1])
            else:
                new_joint_par_ori = R.from_quat(new_joint_orientation[par_idx])
            new_joint_orientation[idx] =new_joint_par_ori * (par_ori.inv() * R.from_quat([joint_orientations[idx]]) )
    return joint_position_new,new_joint_orientation

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
    joint_path_rot,joint_path_pos,joint_path_orientation = ik_pre_data(path,joint_positions,joint_orientations)

    
    return joint_positions, joint_orientations

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