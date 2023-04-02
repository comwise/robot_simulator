#!/bin/bash

set -e

script=$(readlink -f "$0")
route=$(dirname "$script")
upload_dir="ftp://ftp://user:user@ip/xx/softeware/"
is_debug=0

if [ ${is_debug} == 1 ]; then
    route="/home/robot/prj/robot_sim/tools"
    pkg_name="robot_sim_1.0.0_amd64"
else
    pkg_name=$1
fi

echo -e "${GREEN}robot_sim >>> upload deb begin ${NC}"

cd ${route}/../dist

md5sum ${pkg_name}.deb | awk '{ print $1 }' > ${pkg_name}.md5 || { echo `date` ": create md5 file error"; exit 50; }
curl -T "{${pkg_name}.deb,${pkg_name}.md5}" ${upload_dir} || { echo `date` ": upload file to ftp error"; exit 51; }

echo -e "${GREEN}robot_sim >>> upload deb end ${NC}"

exit 0
