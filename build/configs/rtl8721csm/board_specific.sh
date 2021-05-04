#!/usr/bin/env bash
###########################################################################
#
# Copyright 2020 Samsung Electronics All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
# either express or implied. See the License for the specific
# language governing permissions and limitations under the License.
#
###########################################################################
# rtl8721csm_download.sh

## For manual port selection, set $3 as the portname, eg. make download ALL x ttyUSB2
if [ -n "$3" ]; then
	PORT="$3"
else
	PORT="ttyUSB1"
fi

BOARD_NAME="RTL8721CSM"
TOOL_PATH=${CUR_PATH}/../../tools/amebad
IMG_TOOL_PATH=${TOOL_PATH}/image_tool
TOP_PATH=${CUR_PATH}/../../..
SMARTFS_BIN_PATH=${BIN_PATH}/rtl8721csm_smartfs.bin
FLASH_START_ADDR=0x08000000

##Utility function to match partition name to binary name##
function get_executable_name()
{
	case $1 in
		km0_bl) echo "km0_boot_all.bin";;
		km4_bl) echo "km4_boot_all.bin";;
		kernel|ota) echo "km0_km4_image2.bin";;
		userfs) echo "rtl8721csm_smartfs.bin";;
		*) echo "No Binary Match"
		exit 1
	esac
}

##Function to perform board specific initialization##
function board_specific_initialization()
{
	cp -p ${BIN_PATH}/km0_boot_all.bin ${IMG_TOOL_PATH}/km0_boot_all.bin
	cp -p ${BIN_PATH}/km4_boot_all.bin ${IMG_TOOL_PATH}/km4_boot_all.bin
	cp -p ${BIN_PATH}/km0_km4_image2.bin ${IMG_TOOL_PATH}/km0_km4_image2.bin
	if test -f "${SMARTFS_BIN_PATH}"; then
		cp -p ${BIN_PATH}/rtl8721csm_smartfs.bin ${IMG_TOOL_PATH}/rtl8721csm_smartfs.bin
	fi
}

##Download the given partition##
function board_download()
{
	cd ${IMG_TOOL_PATH}
	./amebad_image_tool "download" $1 1 $2 $3
}

##Erase the given partition##
function board_erase()
{
	cd ${IMG_TOOL_PATH}
	./amebad_image_tool "erase" $1 1 $2 0 $3
}
