# Software License Agreement (BSD License)
# Copyright © 2022 belongs to Shadow Robot Company Ltd.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#   1. Redistributions of source code must retain the above copyright notice,
#      this list of conditions and the following disclaimer.
#   2. Redistributions in binary form must reproduce the above copyright notice,
#      this list of conditions and the following disclaimer in the documentation
#      and/or other materials provided with the distribution.
#   3. Neither the name of Shadow Robot Company Ltd nor the names of its contributors
#      may be used to endorse or promote products derived from this software without
#      specific prior written permission.
#
# This software is provided by Shadow Robot Company Ltd "as is" and any express
# or implied warranties, including, but not limited to, the implied warranties of
# merchantability and fitness for a particular purpose are disclaimed. In no event
# shall the copyright holder be liable for any direct, indirect, incidental, special,
# exemplary, or consequential damages (including, but not limited to, procurement of
# substitute goods or services; loss of use, data, or profits; or business interruption)
# however caused and on any theory of liability, whether in contract, strict liability,
# or tort (including negligence or otherwise) arising in any way out of the use of this
# software, even if advised of the possibility of such damage.
from __future__ import absolute_import
import bpy
import os

sr_description_path = '/home/user/projects/shadow_robot/base_deps/src/sr_common/sr_description'

file_names = ['forearm', 'forearm_muscle', 'forearm_muscle_disk', 'forearm_lite', 'wrist', 'palm', 'knuckle',
              'lfmetacarpal', 'F1', 'F2', 'F3', 'TH1_z', 'TH2_z', 'TH3_z']

for file_name in file_names:
    source_file_name = '{0}/meshes/hand/{1}.dae'.format(sr_description_path, file_name)
    dest_file_name = '{0}/mujoco_models/meshes/arm_and_hand_meshes/{1}.stl'.format(sr_description_path, file_name)
    print('Converting {0} to {1}...'.format(source_file_name, dest_file_name))

    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()

    bpy.ops.wm.collada_import(filepath=source_file_name)  # change this line

    bpy.ops.object.select_all(action='SELECT')

    bpy.ops.transform.rotate(value=4.71238898038, axis=(1.0, 0, 0))

    bpy.ops.export_mesh.stl(filepath=dest_file_name)
