import numpy as np
from scipy.spatial.transform import Rotation
from IoHelper import BvhReader
from panda3d.core import Vec3
from panda3d.core import Vec4
import numpy as np
import math

def load_motion_data(bvh_file_path):
    """part2 辅助函数，读取bvh文件"""
    with open(bvh_file_path, 'r') as f:
        lines = f.readlines()
        for i in range(len(lines)):
            if lines[i].startswith('Frame Time'):
                break
        motion_data = []
        for line in lines[i+1:]:
            data = [float(x) for x in line.split()]
            if len(data) == 0:
                break
            motion_data.append(np.array(data).reshape(1,-1))
        motion_data = np.concatenate(motion_data, axis=0)
    return motion_data

def new_load_bvh_data(bvh_file_path):
    reader = BvhReader(bvh_file_path)
    reader.parse()
    return reader.mJointNames,reader.mJointParents,reader.mOffsets,reader.mJointChannels,reader.mJointFrameMotionData

    

def part1_calculate_T_pose(bvh_file_path):
    """请填写以下内容
    输入： bvh 文件路径
    输出:
        joint_name: List[str]，字符串列表，包含着所有关节的名字
        joint_parent: List[int]，整数列表，包含着所有关节的父关节的索引,根节点的父关节索引为-1
        joint_offset: np.ndarray，形状为(M, 3)的numpy数组，包含着所有关节的偏移量

    Tips:
        joint_name顺序应该和bvh一致
    """
    joint_parent = None
    joint_offset = None
    bvhReader = BvhReader(bvh_file_path)
    bvhReader.parse()
    joint_name = bvhReader.mJointNames
    joint_offset = bvhReader.mOffsets
    joint_parent = bvhReader.mJointParents

    return joint_name, joint_parent, joint_offset

def get_joint_pos_and_ori(joint_par_pos,joint_par_ori,joint_channels,frame_data,frame_data_offset):
    parPos = joint_par_pos
    if not parPos:
        parPos = Vec3(0,0,0)
    parOri = joint_par_ori
    if not parPos:
        parPos = Vec3(0,0,0)

def get_rot_and_trans_and_new_offset(joint_channels,joint_motion):
    seq = ""
    ret_pos = Vec3(0,0,0)
    rot_eular = Vec3()
    data_offset = 0
    for channel in joint_channels:
        data = joint_motion
        if channel == "Xposition":
            ret_pos[0] = data[data_offset]
        elif channel == "Yposition":
            ret_pos[1] = data[data_offset]
        elif channel == "Zposition":
            ret_pos[2] = data[data_offset]
        elif channel == "Xrotation":
            seq += "X"
            rot_eular[0] = data[data_offset]
        elif channel == "Yrotation":
            seq += "Y"
            rot_eular[1] = data[data_offset]
        elif channel == "Zrotation":
            seq += "Z"
            rot_eular[2] = data[data_offset]
        data_offset = data_offset + 1
    
    if len(seq)>0:
        ret_rot = Rotation.from_euler(seq,rot_eular,True)
    else:
        ret_rot = Rotation.from_quat([0,0,0,1])    
    return ret_rot,ret_pos


def new_part2_forward_kinematics(joint_name, joint_parent, joint_offset,joint_channels, joint_motions, frameIdx):
    joint_positions = np.empty([len(joint_name),3],dtype=float)
    joint_orientations = []
    joint_orientation_quats =  np.empty([len(joint_name),4],dtype=float)
    assert frameIdx < len(joint_motions) , "Wrong Frame index"
    for jointIdx in range(0,len(joint_name)):
        jointParIdx = joint_parent[jointIdx]
        rot = Rotation.from_quat(Vec4(0,0,0,1))
        pos = Vec3(0,0,0)
        rot,pos = get_rot_and_trans_and_new_offset(joint_channels[jointIdx],joint_motions[frameIdx][jointIdx])
        if jointParIdx != -1:
            ## For Root joint
            rot = (joint_orientations[jointParIdx] * rot)
            ## attention : offset in JOINT is the relative offset against parent joint, so it sould be rot with parent joint quat 
            pos = joint_positions[jointParIdx]+ joint_orientations[jointParIdx].apply(joint_offset[jointIdx])
        else:
            pos = Vec3(joint_offset[jointIdx]) + pos
        joint_positions[jointIdx] = (np.array([pos[0],pos[1],pos[2]]))
        joint_orientations.append(rot)
        rot_quat = rot.as_quat()
        joint_orientation_quats[jointIdx] = np.array([rot_quat[0],rot_quat[1],rot_quat[2],rot_quat[3]])

    return joint_positions, joint_orientation_quats

def part2_forward_kinematics(joint_name, joint_parent, joint_offset, motion_data, frame_id):
    """请填写以下内容
    输入: part1 获得的关节名字，父节点列表，偏移量列表
        motion_data: np.ndarray，形状为(N,X)的numpy数组，其中N为帧数，X为Channel数
        frame_id: int，需要返回的帧的索引
    输出:
        joint_positions: np.ndarray，形状为(M, 3)的numpy数组，包含着所有关节的全局位置
        joint_orientations: np.ndarray，形状为(M, 4)的numpy数组，包含着所有关节的全局旋转(四元数)
    Tips:
        1. joint_orientations的四元数顺序为(x, y, z, w)
        2. from_euler时注意使用大写的XYZ
    """
    joint_positions = None
    joint_orientations = None
    return joint_positions, joint_orientations

def normalize(array_like):
    assert(len(array_like) == 3)
    len = array_like[0] *array_like[0]   + array_like[1] * array_like[1] + array_like[2] * array_like[2]    
    len = math.sqrt(len)
    array_like[0] = array_like[0] / len
    array_like[1] = array_like[1] / len
    array_like[2] = array_like[2] / len
    return array_like

def RotFromAxistoAxis(veca,vecb):
    veca = normalize(veca)
    vecb = normalize(vecb)
    w = 1 + np.dot(veca,vecb)
    xyz = np.cross(veca,vecb) 
    return Rotation.from_quat([xyz[0],xyz[1],xyz[2],w])

def retargetRot(ori_a_to_b,joint_channels_a,joint_channel_data_a,joint_channels_b,joint_channel_data_b,cur_joint_id,par_joint_id):
    seq = ""
    rot_eular = Vec3()
    data_offset = 0
    for channel in joint_channels_a:
        data = joint_channel_data_a
        if channel == "Xrotation":
            seq += "X"
            rot_eular[0] = data[data_offset]
        elif channel == "Yrotation":
            seq += "Y"
            rot_eular[1] = data[data_offset]
        elif channel == "Zrotation":
            seq += "Z"
            rot_eular[2] = data[data_offset]
        data_offset = data_offset + 1
    if len(seq)>0:
        ret_rot = Rotation.from_euler(seq,rot_eular,True)
        ori_a_to_b_par = Rotation.from_quat([0,0,0,1])
        if(par_joint_id != -1) :
            ori_a_to_b_par = ori_a_to_b[par_joint_id]
        ret_rot =ori_a_to_b_par * ret_rot * ori_a_to_b[cur_joint_id].inv()
    else:
        ret_rot = Rotation.from_quat([0,0,0,1])    
    ## Wrote Back
    rot_eular = ret_rot.as_euler(seq)
    data_offset = 0
    for channel in joint_channels_b:
        data = joint_channel_data_b
        if channel == "Xrotation":
            data[data_offset] = rot_eular[0]
        elif channel == "Yrotation":
            data[data_offset] = rot_eular[1]
        elif channel == "Zrotation":
            data[data_offset] = rot_eular[2]
        data_offset = data_offset + 1

def part3_retarget_func(T_pose_bvh_path, A_pose_bvh_path):
    """
    将 A-pose的bvh重定向到T-pose上
    输入: 两个bvh文件的路径
    输出: 
        motion_data: np.ndarray，形状为(N,X)的numpy数组，其中N为帧数，X为Channel数。retarget后的运动数据
    Tips:
        两个bvh的joint name顺序可能不一致哦(
        as_euler时也需要大写的XYZ
    """

    ## Get Pos From A-Pos   

    ret_motion = []
    A_pos_bvh = BvhReader(A_pose_bvh_path)
    A_pos_bvh.parse()
    T_pos_bvh = BvhReader(T_pose_bvh_path)
    T_pos_bvh.parse()
    new_motion_data = np.copy(A_pos_bvh.mJointFrameMotionData)
    rot_a_to_t = [Rotation.from_euler(0,0,0)]
    for i in range(1,len(A_pos_bvh.mJointNames)):
        assert(A_pos_bvh.mJointNames[i] == T_pos_bvh.mJointNames[i])
        ## calc its rotation delta
        a_rot_vec = A_pos_bvh.mOffsets[i] - A_pos_bvh.mOffsets[A_pos_bvh.mJointParents[i]]
        a_rot_vec = normalize(a_rot_vec)
        t_rot_vec = T_pos_bvh.mOffsets[i] - T_pos_bvh.mOffsets[T_pos_bvh.mJointParents[i]]
        t_rot_vec = normalize(t_rot_vec)
        orientation_a = RotFromAxistoAxis([0,1,0],a_rot_vec)
        orientation_b = RotFromAxistoAxis([0,1,0],t_rot_vec)
        ori_a_to_t = orientation_b * orientation_a.inv()
        ## Q_a * R_a_to_t = Q_t 
        rot_a_to_t.append(ori_a_to_t)

        
    A_pos_Offset = A_pos_bvh.mOffsets
    A_pos_Par = A_pos_bvh.mJointParents
    
    for i in range(0,A_pos_bvh.mFrameNum):
        frame_joint_positions, frame_joint_orientation_quats = new_part2_forward_kinematics(A_pos_bvh.mJointNames,A_pos_bvh.mJointParents,A_pos_bvh.mOffsets,A_pos_bvh.mJointChannels,A_pos_bvh.mJointFrameMotionData,i)
        frame_motion = new_motion_data[i]
        for j in range(0,len(frame_joint_positions)):
            joint_data = frame_motion[j]
            retargetRot(rot_a_to_t,T_pos_bvh.mJointChannels[j],T_pos_bvh.mJointFrameMotionData[i][j] ,A_pos_bvh.mJointChannels[j],joint_data,j,A_pos_Par[j])

    motion_data = new_motion_data
    return motion_data
