#!/bin/bash

#set -e

script=$(readlink -f "$0")
route=$(dirname "$script")

route="/home/prj/src/tools"

echo ${script}
echo ${route}

### 1.1 get version
pkg_version=$1
pkg_install_prefix=$2
pkg_name=robot_sim
working_dir=${pkg_name}_dist
pkg_temp_dir=${route}/../../dist/${working_dir}
pkg_tartget_root=/opt/robot
pkg_target_dir=${pkg_tartget_root}/${pkg_name}

session_user=`echo ${SUDO_USER:-$USER}`

echo "session_user ${session_user}"

# 3. change basic file owner
#pkg_files=$(dpkg -L ${pkg_name} | grep ${pkg_target_dir})
#echo ${pkg_files}

uid=`id -un ${session_user}`
gid=`id -gn ${session_user}`

echo "uid="$uid
echo "gid="$gid

pkg_files=$(ls ${pkg_target_dir})

for f in ${pkg_files}; do
    echo "file: "${pkg_target_dir}/$f
    sudo chown ${uid}:${gid} ${pkg_target_dir}/$f
done

echo "test finished"

exit 0
