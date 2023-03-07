#!/usr/bin/bash -l

BASDARD_NAME=`basename ${BASDARD_CONFIG##*/} .conf`
test -n "$BASDARD_PORT" || BASDARD_PORT=`shuf -i 2000-65000 -n 1`
BASDARD_ADAPTER="[NAME=$BASDARD_NAME,PORT=$BASDARD_PORT]"

LVM_ROOT=$HOME
if [ ${LVM_DEBUG} ]; then
    LVM_TAN_CONFIG_PATH=${LVM_ROOT}/lvm/lvmtan/python/lvmtan/config/
    PYTHONPATH=$(ls -1 -d ${LVM_ROOT}/lvm/*/python 2>/dev/null | tr "\n" ":"):$PYTHONPATH
else
    LVM_TAN_CONFIG_PATH=/usr/local/lib/python3.8/dist-packages/lvmtan/python/lvmtan/config/
fi

INSROOT_ETC_PATH=${LVM_TAN_CONFIG_PATH}:${INSROOT_ETC_PATH}
QT_PLUGIN_PATH=${LVM_TAN_CONFIG_PATH}:${QT_PLUGIN_PATH}

if [ ${LVM_RMQ} ]; then
     RMQ_CONNECTION="[user=guest,password=guest,host=${LVM_RMQ},port=5672]"
else
     test -n "${RMQ_CONNECTION}" && {RMQ_CONNECTION0="[user=guest,password=guest,host=localhost,port=5672]"
fi


test -n "$BASDARD_UI" && BASDARD_UI=+UI=$BASDARD_UI

echo $BASDARD_NAME
echo $BASDARD_UI
