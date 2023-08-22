#!/usr/bin/bash -l

PYTHON=/usr/bin/python3

BASDARD_NAME=`basename ${BASDARD_CONFIG##*/} .conf`
test -n "$BASDARD_PORT" || BASDARD_PORT=`shuf -i 2000-65000 -n 1`
BASDARD_ADAPTER="[NAME=$BASDARD_NAME,PORT=$BASDARD_PORT]"

BASDARD_LOGFILE="/data/logs/lvmtan/basdard_$(date +%Y-%m-%d_%H:%M:%S).log"

if [ ${LVM_DEBUG} ]; then
    LVM_TAN_CONFIG_PATH=${LVM_ROOT}/python/lvmtan/config/
    PYTHONPATH=${LVM_ROOT}/python:$PYTHONPATH
else
    LVM_TAN_CONFIG_PATH=$(${PYTHON} -c "import lvmtan as _; print(_.__path__[0])")/config
fi

INSROOT_ETC_PATH=${LVM_TAN_CONFIG_PATH}:${INSROOT_ETC_PATH}
QT_PLUGIN_PATH=${LVM_TAN_CONFIG_PATH}:${QT_PLUGIN_PATH}

if [ ${LVM_RMQ} ]; then
     RMQ_CONNECTION="[user=guest,password=guest,host=${LVM_RMQ},port=5672]"
else
     test -n "${RMQ_CONNECTION}" && {RMQ_CONNECTION="[user=guest,password=guest,host=localhost,port=5672]"
fi


test -n "$BASDARD_UI" && BASDARD_UI=+UI=$BASDARD_UI

echo $BASDARD_NAME
echo $BASDARD_UI
echo $LVM_TAN_CONFIG_PATH
