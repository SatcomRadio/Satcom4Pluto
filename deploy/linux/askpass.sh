#!/bin/sh
# answers ssh's password prompt for the Pluto (password from /etc/satcom4pluto.conf, default "analog")
echo "${PLUTO_PASSWORD:-analog}"
