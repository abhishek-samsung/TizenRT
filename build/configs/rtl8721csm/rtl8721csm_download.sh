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



CURDIR=$(readlink -f "$0")
CUR_PATH=$(dirname "$CURDIR")
BIN_PATH=${CUR_PATH}/../../output/bin
TOP_PATH=${CUR_PATH}/../../..
OS_PATH=${TOP_PATH}/os

source ${TOP_PATH}/build/configs/rtl8721csm/board_specific.sh

WARNING="\n Port $PORT is selected\n\n
        ############################################\n
        WARNINGS:\n
        1. Make sure the board is in DOWNLOAD MODE.\n
        2. Make sure NO other application like putty,\n
        is occupying $PORT.\n
        ############################################\n"

echo -e $WARNING

TTYDEV="/dev/${PORT}"
CONFIG=${OS_PATH}/.config
source ${CONFIG}

##Utility function for sanity check##
function sanity_check()
{
	if [ ! -f ${CONFIG} ];then
		echo "No .config file"
		exit 1
	fi

	if [[ "${CONFIG_ARCH_BOARD_${BOARD_NAME}}" != "y" ]];then
		echo "Target is NOT ${BOARD_NAME}"
		exit 1
	fi

	if fuser -s ${TTYDEV};then
		echo "${TTYDEV} is used by another process, can't proceed"
		exit 1
	fi
}

flash_ota=false
##Utility function to get partition index ##
function get_partition_index()
{
	for idx in ${!parts[@]}; do
		if [[ ${parts[$idx],,} == ${1,,} ]]; then
			if [[ $flash_ota == true ]]; then
				## In this case, try to find 2nd kernel partition for OTA
				flash_ota=false
				continue;
			fi
			echo $idx
			exit 1
		fi
	done
	echo -1
}

##Help utility##
function dwld_help()
{
        cat <<EOF
	HELP:
		make download ALL or [PARTITION(S)]
		make download ERASE ALL or erase [PARTITION(S)]

	PARTITION(S):
		[${uniq_parts[@]}]  NOTE:case sensitive

	For examples:
		make download ALL
		make download ERASE ALL
		make download kernel
		make download erase kernel
		make download ota
		make download erase ota
		make download erase ss
		make download smartfs
		make download erase userfs
EOF
}

##Utility function to read partitions from .config or Kconfig##
function get_configured_partitions()
{
	local configured_parts
	# Read Partitions
	if [[ -z ${CONFIG_FLASH_PART_NAME} ]];then

		configured_parts=`grep -A 2 'config FLASH_PART_NAME' ${PARTITION_KCONFIG} | sed -n 's/\tdefault "\(.*\)".*/\1/p'`
	else
		configured_parts=${CONFIG_FLASH_PART_NAME}
	fi

	echo $configured_parts
}

##Utility function to get the partition sizes from .config or Kconfig
function get_partition_sizes()
{
	local sizes_str
	#Read Partition Sizes
	if [[ -z ${CONFIG_FLASH_PART_SIZE} ]]
	then
		sizes_str=`grep -A 2 'config FLASH_PART_SIZE' ${PARTITION_KCONFIG} | sed -n 's/\tdefault "\(.*\)".*/\1/p'`
	else
		sizes_str=${CONFIG_FLASH_PART_SIZE}
	fi

	echo $sizes_str
}

# Start here

board_specific_initialization;

sanity_check;

parts=$(get_configured_partitions)
IFS=',' read -ra parts <<< "$parts"

sizes=$(get_partition_sizes)
IFS=',' read -ra sizes <<< "$sizes"

#Calculate Flash Offset
num=${#sizes[@]}
offsets[0]=`printf "0x%X" ${FLASH_START_ADDR}`

for (( i=1; i<=$num-1; i++ ))
do
	val=$((sizes[$i-1] * 1024))
	offsets[$i]=`printf "0x%X" $((offsets[$i-1] + ${val}))`
done

#Dump Info
echo ""
echo "================================ < Flash Partition Information > ================================"
printf "\rNAME       :"
printf '  %12s' "${parts[@]}"
printf "\r\nSIZE(in KB):"
printf '  %12s' "${sizes[@]}"
printf "\r\nAddr       :"
printf '  %12s' "${offsets[@]}"
printf "\n"
echo "================================================================================================="
echo ""

if test $# -eq 0; then
	echo "FAIL!! NO Given partition. Refer \"PARTITION NAMES\" above."
	rtl8721csm_dwld_help 1>&2
	exit 1
fi

uniq_parts=($(printf "%s\n" "${parts[@]}" | sort -u));

#Validate arguments
for i in ${cmd_args[@]};do

	if [[ "${i}" == "ALL" ]];then
		continue;
	fi

	for j in ${uniq_parts[@]};do
		if [[ "${i}" == "${j}" ]];then
			result=yes
		fi
	done

	if [[ "$result" != "yes" ]];then
		echo "FAIL!! Given \"${i}\" partition is not available. Refer \"PARTITION NAMES\" above."
		dwld_help
		exit 1
	fi
	result=no
done

download_specific_partition()
{
	TARGET=$1
	if [[ ${TARGET} == "ota" || ${TARGET} == "OTA" ]];then
		TARGET="kernel"
		flash_ota=true
	fi
	partidx=$(get_partition_index ${TARGET})
	if [[ "${partidx}" < 0 ]];then
		echo "Not supported"
		rtl8721csm_dwld_help
		exit 1
	fi

	# Get a filename and Download a file
	echo ""
	echo "============================="
	if [[ $1 == "ota" || $1 == "OTA" ]];then
		echo "Downloading Kernel OTA binary"
	else
		echo "Downloading ${parts[$partidx]} binary"
	fi
	echo "============================="
	exe_name=$(get_executable_name ${parts[$partidx]})
	board_download $TTYDEV ${offsets[$partidx]} ${exe_name}

	echo ""
	echo "Download $exe_name COMPLETE!"
}

download_all()
{
	cd ${IMG_TOOL_PATH}
	echo "Starting Download..."
	found_kernel=false

	for partidx in ${!parts[@]}; do

		if [[ "${CONFIG_APP_BINARY_SEPARATION}" != "y" ]];then
			if [[ "${parts[$partidx]}" == "userfs" ]];then
				continue
			fi
		fi

		if [[ "${parts[$partidx]}" == "kernel" ]];then
			if [[ $found_kernel == true ]];then
				continue
			fi
			found_kernel=true
		fi

		exe_name=$(get_executable_name ${parts[$partidx]})
		[ "No Binary Match" = "${exe_name}" ] && continue

		echo ""
		echo "=========================="
		echo "Downloading ${parts[$partidx]} binary"
		echo "=========================="

		board_download $TTYDEV ${offsets[$partidx]} ${exe_name}
	done
	echo ""
	echo "Download COMPLETE!"
}

erase()
{
	echo "Starting Erase..."
	ota_addr=$(($FLASH_START_ADDR + $CONFIG_AMEBAD_FLASH_CAPACITY))
	if [[ $2 == "all" || $2 == "ALL" ]];then
		found_kernel=false
		flash_ota=false
		echo ""
		echo "=========================="
		echo "      Erasing All"
		echo "=========================="
		for partidx in ${!parts[@]}; do
			if [[ "${parts[$partidx]}" == "userfs" ]];then
				continue
			elif [[ "${parts[$partidx]}" == "ss" ]];then
				continue
			else
				echo ""
				echo "=========================="
				echo "Erasing ${parts[$partidx]} partition"
				echo "=========================="
			fi

			board_erase $TTYDEV ${offsets[$partidx]} ${sizes[partidx]}
		done
	else
		for partidx in ${!parts[@]}; do
			if [[ $2 == "kernel" || $2 == "KERNEL" ]];then
				if [[ "${parts[$partidx]}" != "kernel" ]];then
					continue
				elif [[ ${offsets[$partidx]} -lt ${ota_addr} ]];then
					ota_addr=0
					found_kernel=false
				else
					continue
				fi
			elif [[ $2 == "ota" || $2 == "OTA" ]];then
				if [[ "${parts[$partidx]}" != "kernel" ]];then
					continue
				elif [[ ${offsets[$partidx]} -lt ${ota_addr} ]];then
					ota_addr=${offsets[$partidx]}
					continue
				else
					flash_ota=false
				fi
			elif [[ $2 == "ss" || $2 == "SS" ]];then
				if [[ "${parts[$partidx]}" != "ss" ]];then
					continue
				fi
			elif [[ $2 == "userfs" || $2 == "USERFS" ]];then
				if [[ "${parts[$partidx]}" != "userfs" ]];then
					continue
				fi
			else
				printf "\n## Invalid Option ##\n\n"
				dwld_help
				break
			fi

			echo ""
			echo "=========================="
			echo "Erasing ${parts[$partidx]} partition"
			echo "=========================="
			board_erase $TTYDEV ${offsets[$partidx]} ${sizes[partidx]}
		done
	fi
	echo ""
	echo "Erase COMPLETE!"
}

case $1 in
	all|ALL)
		download_all
		;;
	erase|ERASE)
		erase $1 $2
		;;
	*)
		download_specific_partition $1
		;;
esac

[ -e km0_boot_all.bin ] && rm km0_boot_all.bin
[ -e km4_boot_all.bin ] && rm km4_boot_all.bin
[ -e km0_km4_image2.bin ] && rm km0_km4_image2.bin
if test -f "${SMARTFS_BIN_PATH}"; then
	[ -e rtl8721csm_smartfs.bin ] && rm rtl8721csm_smartfs.bin
fi
