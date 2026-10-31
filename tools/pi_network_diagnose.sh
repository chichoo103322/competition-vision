#!/usr/bin/env bash
# Read-only diagnostics; never display Wi-Fi passwords or connection secrets.
export LC_ALL=C
printf '\n--- identity and boot ---\n'
hostname
uptime
who -b
printf '\n--- services ---\n'
systemctl is-active NetworkManager ssh avahi-daemon
systemctl is-enabled NetworkManager ssh avahi-daemon
printf '\n--- current interfaces ---\n'
nmcli -f DEVICE,TYPE,STATE,CONNECTION device status
ip -4 address show
ip -4 route show
printf '\n--- saved connection summary (no passwords) ---\n'
nmcli -f NAME,UUID,TYPE,AUTOCONNECT,AUTOCONNECT-PRIORITY connection show
wifi_device=$(nmcli -t -f DEVICE,TYPE device status | awk -F: '$2=="wifi" {print $1;exit}')
if [ -n "$wifi_device" ]; then
    printf '\n--- active Wi-Fi profile and power saving ---\n'
    wifi_connection=$(nmcli -g GENERAL.CONNECTION device show "$wifi_device")
    if [ -n "$wifi_connection" ] && [ "$wifi_connection" != '--' ]; then
        nmcli -f connection.id,connection.uuid,connection.autoconnect,connection.autoconnect-priority,connection.autoconnect-retries,connection.permissions,802-11-wireless.ssid,802-11-wireless.powersave,ipv4.method connection show "$wifi_connection"
    fi
    if command -v iw >/dev/null; then
        iw dev "$wifi_device" link
        iw dev "$wifi_device" get power_save
    fi
fi
printf '\n--- radio and power health ---\n'
rfkill list
if command -v vcgencmd >/dev/null; then vcgencmd get_throttled; fi
printf '\n--- listening ports ---\n'
ss -ltn
printf '\n--- NetworkManager current boot (last 180 entries) ---\n'
journalctl -b -u NetworkManager --no-pager -n 180 -o short-monotonic
printf '\n--- NetworkManager previous boot (last 70 entries) ---\n'
journalctl -b -1 -u NetworkManager --no-pager -n 70 -o short-monotonic
printf '\n--- kernel Wi-Fi / USB / voltage events ---\n'
journalctl -b -k --no-pager -n 1500 | rg -i 'brcm|wlan|wifi|under.?voltage|voltage|disconnect|usb.*reset' 2>/dev/null ||
    journalctl -b -k --no-pager -n 1500 | grep -Ei 'brcm|wlan|wifi|under.?voltage|voltage|disconnect|usb.*reset'
printf '\n--- diagnostic complete ---\n'
