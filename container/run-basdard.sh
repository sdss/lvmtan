#!/usr/bin/bash -l

. /root/env-basdard.sh

start_niceui() {
   sleep 1.2 && niceQUI --XXX.CONFIG:Endpoint=${BASDARD_ADAPTER}${BASDARD_UI} --LOGGER.LEVEL=INFO
}

max_niceui() {
    while [[ -z $(wmctrl -l) ]]; do sleep 0.1; done
    wmctrl -r ':ACTIVE:' -b toggle,fullscreen
}

if [ $VNC_GEOM ]; then
    Xvnc :0 -geometry $VNC_GEOM &
    export DISPLAY=:0
    fluxbox &
    start_niceui &
    max_niceui &
    (cd /usr/share/novnc/ && ~/novnc_server &)
elif [ ${DISPLAY} ]; then
    start_niceui &
fi

basdard --CONFIG=${BASDARD_CONFIG} --ADAPTER=${BASDARD_ADAPTER} --CLU.RABBITMQ.CONN:MapStringString=${RMQ_CONNECTION} --LOGGER.LEVEL=${BASDARD_LOG_LEVEL:=INFO} | tee ${BASDARD_LOGFILE}


if [ $LVM_DEBUG ]; then 
   sleep INFINITY
fi
