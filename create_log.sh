#!/usr/bin/env bash

# Dieses Skript erstellt die Syslog-Regeln für den Log.
# WICHTIG: Das Skript sollte als root ausgeführt werden. Ein "sudo" reicht nicht.
# Die Logs werden danach automatisch in /var/log/babocam.log gespeichert.
# Per 'tail -F /var/log/babocam.log' lassen sich die Logs in Echtzeit mitlesen.
# Mit '> /var/log/babocam.log' können die Logs geleert werden.

echo "local0.info /var/log/babocam.log" >> /etc/syslog/conf
touch /var/log/babocam.log
chown syslog.adm /var/log/babocam.log
chmod 666 /var/log/babocam.log