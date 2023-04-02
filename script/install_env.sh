#!/bin/bash

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'
BOLD='\033[1m'

echo -e "${GREEN}${BOLD}--- robot_sim runtime begin ----${NC}"

## 1.0 environment variable
script=$(readlink -f "$0")
route=$(dirname "$script")
lib_server="ftp://user:user@ip/xx/lib"
deb_pkgs=()

mkdir -p ~/.tmp
tmp_dir=~/.tmp

## 2. install lib
apt_pkgs+=(
    # log
    libspdlog-dev

    #jsoncpp
    libjsoncpp-dev

    # pack app tool
    fakeroot

    # run app tools
    circus
)

for n in ${apt_pkgs[@]}
do
    echo -e "${RED}Processing dpkg pkg ----> " $n ${NC}
    sudo apt-get install -y $n || { fail_dpkgs+=($n); continue; }
    echo -e "${RED}Processing dpkg pkg ----> " $n " $BOLD OK" ${NC}
done

## 3 install other lib
deb_pkgs+=(
)
for n in ${deb_pkgs[@]}
do
    echo -e "${RED}Processing deb ----> " $n ${NC}
    [ -f "${tmp_dir}/${n}" ] || { wget -P ${tmp_dir} ${lib_server}/${n} || { fail_debs+=($n); continue; } }
    sudo dpkg -E -i ${tmp_dir}/${n} || { echo -e "${RED}Fail installing debs: $BOLD ${fail_debs[@]}" ${NC}; continue; }
    echo -e "${RED}Processing deb----> " $n "$BOLD OK" ${NC}
done

echo -e "${GREEN}${BOLD}--- robot_sim runtime end ----${NC}"

exit 0
