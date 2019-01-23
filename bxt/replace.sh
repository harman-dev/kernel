#! /bin/bash
set -x

read -r -p "Enter the delivery JIRA ticket for kernel: " response
export TRACKED_ON="Tracked-On: $response"

for patchfile in *.patch; do
    sed -i 's/Tracked-On/Original-Tracked-On/' $patchfile
    awk -v jira_ticket="$TRACKED_ON" ' { print; if ( match($0, "Change-Id") ) print jira_ticket; }' $patchfile > tmpfile
    mv tmpfile $patchfile
done
