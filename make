#!/bin/bash
export TFTP=/home/workspace/micron/00_job_EBU/tftp
if make -j6 #& make modules_install INSTALL_MOD_PATH=/workspace/fs/mountfs/fs6410/filesystem/
 then
     
   if make ARCH=arm UIMAGE_LOADADDR=0x8000 uImage -j8
	then
	 echo "cp object image to $TFTP"
	   if cp ./arch/arm/boot/uImage $TFTP/ && cp ./arch/arm/boot/zImage $TFTP/
		then
			if cp ./arch/arm/boot/dts/zynq-zed.dtb $TFTP/devicetree.dtb
				then
					echo "**** copy uImage and dtb OK! ****"
			else
				echo "**** copy dtb fail! ****"
			fi
	    else
			echo "**** copy uImage error!****"
	    fi
   else
	echo "make uImage error!!!!!!"
   fi
else
 echo "make failed !"
fi
