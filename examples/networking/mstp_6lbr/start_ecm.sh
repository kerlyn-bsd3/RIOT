#!/usr/bin/env bash
# Host side of the mstp_6lbr UPLINK=cdc-ecm build: bring up the 6LBR's USB
# Ethernet (cdc_ether) interface and route the MS/TP prefix through it.
# Mirrors dist/tools/sliptty/start_network.sh: host fe80::1 (+ fdea:dbee:f::1/64),
# 6LBR fe80::2, PREFIX via fe80::2. Return/Ctrl-C removes what it added.
#
# usage: sudo ./start_ecm.sh [prefix]        (default aaaa::/64)
#        sudo IFACE=enx... ./start_ecm.sh     (pick an interface explicitly)
set -u
PREFIX=${1:-aaaa::/64}
HOST_GLB=${HOST_GLB:-fdea:dbee:f::1/64}

# The 6LBR's interface is the cdc_ether netdev whose USB device reports the
# RIOT manufacturer string (the monitor is CDC-ACM, so it never matches).
find_iface() {
    for n in /sys/bus/usb/drivers/cdc_ether/*/net/*; do
        [ -e "$n" ] || continue
        dev=$(readlink -f "$n/device/..")
        if grep -qi riot "$dev/manufacturer" 2>/dev/null; then
            basename "$n"; return 0
        fi
    done
    return 1
}

IFACE=${IFACE:-}
for _ in 1 2 3 4 5 6 7 8 9 10; do
    [ -n "$IFACE" ] && break
    IFACE=$(find_iface) && break
    IFACE=""
    sleep 1
done
[ -z "$IFACE" ] && { echo "no RIOT cdc_ether interface found (flashed UPLINK=cdc-ecm? nRF USB cable in?)"; exit 1; }
echo "6LBR uplink interface: $IFACE"

cleanup() {
    echo "cleaning up $IFACE"
    ip -6 route del "$PREFIX" via fe80::2 dev "$IFACE" 2>/dev/null
    ip -6 addr del "$HOST_GLB" dev "$IFACE" 2>/dev/null
    ip -6 addr del fe80::1/64 dev "$IFACE" 2>/dev/null
}
trap cleanup EXIT INT TERM

sysctl -qw net.ipv6.conf.all.forwarding=1
sysctl -qw net.ipv6.conf."$IFACE".accept_ra=0
ip link set "$IFACE" up
ip -6 addr add fe80::1/64 dev "$IFACE"
ip -6 addr add "$HOST_GLB" dev "$IFACE"
ip -6 route add "$PREFIX" via fe80::2 dev "$IFACE"

echo "routes:"; ip -6 route show dev "$IFACE"
echo "try: ping6 -c 3 fe80::2%$IFACE ; ping6 aaaa::ff:fe00:3"
echo "press Return to tear down"
read -r _
