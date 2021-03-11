#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
TARGET_RULE_FILE="roborts.rules"
TARGET_DEV_MODE="777"
TARGET_DEV_NAME="ttyACM.*[0-9]"
TARGET_DEV_LINK="serial_sdk"
TARGET_ID_VENDOR="0483"
TARGET_ID_PRODUCT="5740"
UDEV_RULES="KERNEL==\"${TARGET_DEV_NAME::6}*\", ATTRS{idVendor}==\"${TARGET_ID_VENDOR}\", ATTRS{idProduct}==\"${TARGET_ID_PRODUCT}\", MODE:=\"${TARGET_DEV_MODE}\", SYMLINK+=\"${TARGET_DEV_LINK}\""

CHECK_UDEV_FILE(){
  if [ -f "/etc/udev/rules.d/${TARGET_RULE_FILE}" ]; then
    GET_UDEV_RULES=$(cat "/etc/udev/rules.d/${TARGET_RULE_FILE}")
    if [[ "${GET_UDEV_RULES}" =~ "${UDEV_RULES}" ]]
    then
      echo "[INFO]  udev file has been configured."
      return 0
    else
      echo "[INFO]  udev file has been modified, where current rules: "
      echo "${GET_UDEV_RULES}"
      echo "should contain: "
      echo "${UDEV_RULES}"
      return 2
    fi
  else
    echo "[INFO]  udev file does not exist"
    return 1
  fi
}

RELOAD_UDEV(){
  echo "[INFO]  Reload udev rules configuration"
  sudo udevadm control --reload-rules
  sudo udevadm trigger
}

CREATE_UDEV_FILE(){
  echo "[INFO]  Add udev rules file to /etc/udev/rules.d/${TARGET_RULE_FILE} with content:"
  echo ""
  echo "${UDEV_RULES}" |sudo tee "/etc/udev/rules.d/${TARGET_RULE_FILE}"
  echo ""
}

DELETE_UDEV_FILE(){
  echo "[INFO]  Delete udev rules file /etc/udev/rules.d/${TARGET_RULE_FILE}"
  sudo rm  "/etc/udev/rules.d/${TARGET_RULE_FILE}"
}

CREATE_UDEV(){
  echo "[INFO]  Remap the device port(ttyACM*) to ${TARGET_DEV_LINK}"
  CHECK_UDEV_FILE
  case $? in
  "0")
    echo "[ERROR]  Udev rules file already exists and no need copying."
    return 1
    ;;
  "1")
    CREATE_UDEV_FILE
    RELOAD_UDEV
    return 0
    ;;
  "2")
    while true
    do
      read -r -p "[WARN]  Udev rules already exists but modified. Are You sure to override existing file? [Y/n] " input
      case $input in
      [yY][eE][sS]|[yY])
        CREATE_UDEV_FILE
        RELOAD_UDEV
        return 0
        ;;

      [nN][oO]|[nN])
        echo "[ERROR] Failed to override existing file."
        return 2
        ;;

      *)
        echo "[ERROR] Invalid input. please input [Y/n]"
        ;;
      esac
    done
  esac
}

DELETE_UDEV(){
  echo "[INFO]  Cancel remapping the device port(ttyACM*) to ${TARGET_DEV_LINK}"
  CHECK_UDEV_FILE
  case $? in
  "0")
    DELETE_UDEV_FILE
    RELOAD_UDEV
    return 0
    ;;
  "1")
    echo "[WARN]  Udev rules file not exists and no need deleting "
    return 1
    ;;
  "2")
    while true
    do
      read -r -p "[WARN]  Udev rules already exists but modified. Are You sure to delete existing file? [Y/n] " input
      case $input in
      [yY][eE][sS]|[yY])
        DELETE_UDEV_FILE
        RELOAD_UDEV
        return 0
        ;;

      [nN][oO]|[nN])
        echo "[ERROR] Failed to delete existing file."
        return 2
        ;;

      *)
        echo "[ERROR] Invalid input. please input [Y/n]"
        ;;
      esac
    done
  esac

}

CHECK_UDEV_STATUS(){
  echo "[INFO]  Check USB device connection and udev status"
  DEVICE_NUM=0
  UDEV_FLAG=0
  for DEV_SYS_PATH in $(find /sys/bus/usb/devices/usb*/ -name dev | grep ${TARGET_DEV_NAME}/dev)
  do
    DEV_SYS_PATH="${DEV_SYS_PATH%/dev}"
    eval "$(udevadm info -q property --export -p ${DEV_SYS_PATH})" 2>/dev/null
    if [[ (${ID_VENDOR_ID} == ${TARGET_ID_VENDOR}) &&
          (${ID_MODEL_ID} == ${TARGET_ID_PRODUCT}) &&
          ($(echo ${DEVNAME} | grep ${TARGET_DEV_NAME})) ]]
    then
      DEVICE_NUM=$(($DEVICE_NUM+1))
      if [[ (${DEVLINKS} =~ ${TARGET_DEV_LINK}) ]]
      then
        UDEV_FLAG=1
        echo "[SUCCEED]  USB device connected and udev rules is configured: ${DEVNAME}->/dev/${TARGET_DEV_LINK}"
      else
        CHECK_UDEV_FILE
        if [ $? -ne 0 ]
        then
          echo "[ERROR]  USB device connected but udev rules is not configured correctly: ${DEVNAME}->null "
          echo "         Please run "
          echo "                 $0 create "
          echo "         to create udev rules for device "
        fi
      fi
    fi
  done
  if [ $DEVICE_NUM -eq 0 ]
  then
    echo "[ERROR]  No device has been connected"
    return 1
  elif [ $DEVICE_NUM -eq 1 ]
  then
    echo "[INFO]  Check process finshed with $DEVICE_NUM device connected"
    if [ $UDEV_FLAG -eq 1 ]
    then
      return 0
    else
      return 1
    fi
  else
    echo "[ERROR]  Check process finshed with $DEVICE_NUM devices connected,
    you can only connect one device while using"
    return 1
  fi
}

HELP(){
  echo "
  $0 [ARGS]create/delete/check/status
  ARGUMETNS:
  create:  create udev configuration
  delete:  delete udev configuration
  check:   check udev configuration
  status:  get usb connection and udev status
  "
  return 0
}

case $1 in
   "create")
       CREATE_UDEV
       ;;
   "delete")
       DELETE_UDEV
       ;;
   "check")
       CHECK_UDEV_FILE
       ;;
   "status")
       CHECK_UDEV_STATUS
       ;;
   *)
       HELP
       ;;
esac
if [ $? -eq 0 ]
then
  exit 0
else
  exit 1
fi