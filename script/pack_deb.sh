#!/bin/bash

set -e
echo -e "${GREEN}robot_sim >>>  pack deb begin ${NC}"

### 1.0 ready job
script=$(readlink -f "$0")
route=$(dirname "$script")

pkg_version=$1
pkg_install_prefix=$2
pkg_name=robot_sim
working_dir=${pkg_name}_dist
pkg_temp_dir=${route}/../dist/${working_dir}
pkg_tartget_root=/opt/robot
pkg_target_dir=${pkg_tartget_root}/${pkg_name}
pkg_file=robot-sim

### 1.1 arch
uname_arch=$(uname -m)
if [ x"${uname_arch}" == x"x86_64" ]; then
    arch=amd64
elif [ x"${uname_arch}" == x"aarch64" ]; then
    arch=arm64
else
    echo "not support arch ${uname_arch}" >&2
    exit 2
fi

### 1.2 vesion
if [ "${pkg_version}" == "" ]; then
    pkg_version=$(git describe --tag) > /dev/null 2>&1 || { echo -e "${RED}get git tag is null ${NC}"; }
    if [ "${pkg_version}" == "" ]; then
        pkg_version=$(cat ${route}/../version)
        echo "can't get version from gitlab, so use version from '{project}/version': ${pkg_version}"
    else
        echo ${pkg_version} > ${route}/../version
    fi
fi
echo "deb version is ${pkg_version}"

### 1.3 get install dir
if [ "${pkg_install_prefix}" == "" ]; then
    pkg_install_prefix=${pkg_tartget_root}
fi
echo "deb install prefix is ${pkg_install_prefix}"

### 1.4 install dir 
which cmake > /dev/null 2>&1 || { echo -e "${RED}cmake no found ${NC}"; exit 13; }
if [ ! -d ${pkg_target_dir} ]; then
    sudo mkdir -p ${pkg_target_dir} > /dev/null 2>&1 || { echo "create ${pkg_name} dir(${pkg_target_dir}) error"; exit 14; }
fi

session_user=`echo ${SUDO_USER:-$USER}`
uid=`id -un ${session_user}`
gid=`id -gn ${session_user}`
sudo chown -R ${uid}:${gid} ${pkg_target_dir}

### 2.0 make install
mkdir -p ${route}/../build || { echo -e "mkdir build failed"; exit 20; }
cd ${route}/../build
cmake .. && make

### 2.1 crete deb dir
bk_idx=1
if [ -e ${pkg_temp_dir} ]; then
    while [ -e ${pkg_temp_dir}.${bk_idx} ]; do
        let bk_idx++
    done
    mv ${pkg_temp_dir} ${pkg_temp_dir}.${bk_idx}
fi
mkdir -p ${pkg_temp_dir}
mkdir -p ${pkg_temp_dir}/DEBIAN
mkdir -p ${pkg_temp_dir}/${pkg_install_prefix}/${pkg_name}

### 2.2 copy targets to deb ready dir
#rsync -a ${route}/../install/* ${pkg_temp_dir}/${pkg_install_prefix}/${pkg_name}
#cp -rf ${route}/../install/* ${pkg_temp_dir}/${pkg_install_prefix}/${pkg_name}
sudo mv -T ${pkg_target_dir}/ ${pkg_temp_dir}/${pkg_install_prefix}/${pkg_name}
mkdir -p ${pkg_temp_dir}/${pkg_install_prefix}/${pkg_name}/script
mkdir -p ${pkg_temp_dir}/${pkg_install_prefix}/${pkg_name}/bin
mkdir -p ${pkg_temp_dir}/${pkg_install_prefix}/${pkg_name}/devel
chmod +x ${route}/../script/*.sh

#cp -rf ${route}/depend ${pkg_temp_dir}/${pkg_install_prefix}/${pkg_name}/
cp -rf ${route}/../bin/sim_node ${pkg_temp_dir}/${pkg_install_prefix}/${pkg_name}/bin
cp -rf ${route}/../build/devel/* ${pkg_temp_dir}/${pkg_install_prefix}/${pkg_name}/devel
cp -rf ${route}/install_env.sh ${pkg_temp_dir}/${pkg_install_prefix}/${pkg_name}/script
cp -rf ${route}/run_node.sh ${pkg_temp_dir}/${pkg_install_prefix}/${pkg_name}/script
cp -rf ${route}/cp.sh ${pkg_temp_dir}/${pkg_install_prefix}/${pkg_name}/script
cp -rf ${route}/../changelog ${pkg_temp_dir}/${pkg_install_prefix}/${pkg_name}/
cp -rf ${route}/../version ${pkg_temp_dir}/${pkg_install_prefix}/${pkg_name}/

## 2.3 create deb config files under DEBIAN dir
cd ${pkg_temp_dir}/DEBIAN
touch control
(cat << EOF
Package: ${pkg_file}
Version: ${pkg_version}
Section: x11
Priority: optional
Depends:
Suggests:
Architecture: ${arch}
Maintainer: comwise softerware group
CopyRight: commercial
Provider: comwise Corp.
Description: comwise simulator package
EOF
) > control

touch postinst
(cat << EOF
#!/bin/bash
session_user=\`echo \${SUDO_USER:-\$USER}\`

# 1. circus conf
mkdir -p /etc/circus/conf.d
(cat << EOF2
[watcher:${pkg_name}]
cmd=/bin/bash
args=${pkg_target_dir}/script/run_node.sh
working_dir=${pkg_target_dir}
uid=\`id -un \${session_user}\`
gid=\`id -gn \${session_user}\`
autostart=True
numprocesses=1
stop_signal=SIGINT
stop_children=True
max_retry=-1
graceful_timeout=30
priority=0
singleton=True
respawn=True
stdout_stream.class=FileStream
stdout_stream.filename=/dev/null
stderr_stream.class=FileStream
stderr_stream.filename=/opt/cotek/log/${pkg_name}_err.log
stderr_stream.max_bytes=1048576
stderr_stream.backup_count=2
stderr_stream.time_format=%Y-%m-%d %H:%M:%S
EOF2
) > /etc/circus/conf.d/${pkg_name}.ini

# 2. bin shortcut
ln -fs ${pkg_target_dir}/script/run_node.sh /usr/bin/${pkg_name}

# 3. change basic file owner
pkg_files=\$(dpkg -L ${pkg_file} | grep ${pkg_target_dir})
uid=\`id -un \${session_user}\`
gid=\`id -gn \${session_user}\`
chown -R \${uid}:\${gid} \${pkg_files}
# 4. set special setuid
ldconfig

# 4. depend lib
ldconfig

# 5. log
sudo mkdir -p ${pkg_tartget_root}/log/
sudo chown -R \${uid}:\${gid} ${pkg_tartget_root}/log/

EOF
) > postinst

touch postrm
(cat << EOF
#!/bin/bash
EOF
) > postrm

touch preinst
(cat << EOF
#!/bin/bash
EOF
) > preinst

touch prerm
(cat << EOF
#!/bin/bash
rm -rf /usr/bin/${pkg_name}
rm -rf ${pkg_target_dir}/build
rm -rf ${pkg_target_dir}/devel
rm -rf ${pkg_target_dir}/.catkin_workspace
rm -rf ${pkg_target_dir}
rm -rf /etc/circus/conf.d/${pkg_name}.ini
EOF
) >> prerm
chmod +x postinst postrm preinst prerm

## 3 start to pack deb
cd ${route}/../
deb_file=${pkg_name}_${pkg_version}_${arch}
fakeroot dpkg -b dist/${working_dir} dist/${deb_file}.deb || exit 30
rm -rf dist/${working_dir}

#${route}/upload_deb.sh ${deb_file}

echo -e "${GREEN}robot_sim >>> pack ${deb_file}.deb finished ${NC}"

exit 0
