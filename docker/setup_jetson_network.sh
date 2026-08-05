#!/usr/bin/env bash
# Prepares THIS machine's ROS2/DDS network so the sim (running natively or
# inside the rc_sim container, which uses network_mode: host so it shares
# this machine's network stack) can discover and exchange topics with the
# ROS2 Foxy stack on the Jetson Xavier over a direct Ethernet link.
#
# IMPORTANT: run this with `source`, not `./`, so the exported env vars
# survive in your current shell (a plain `./` runs it in a subshell and the
# exports are lost the moment the script exits):
#
#   source docker/setup_jetson_network.sh <jetson_ip> [ros_domain_id]
#
# The same ROS_DOMAIN_ID must be set on the Jetson side too, and its
# firewall (if enabled) must allow traffic from this machine's subnet.

if [ -z "$1" ]; then
    echo "Usage: source ${BASH_SOURCE[0]} <jetson_ip> [ros_domain_id]"
    return 1 2>/dev/null || exit 1
fi

JETSON_IP="$1"
DOMAIN_ID="${2:-0}"

echo "== 1) RMW / DDS env =="
if [ -n "$RMW_IMPLEMENTATION" ] && [ "$RMW_IMPLEMENTATION" != "rmw_fastrtps_cpp" ]; then
    echo "  RMW_IMPLEMENTATION was '$RMW_IMPLEMENTATION' - clearing it."
    echo "  (Foxy's default is Fast DDS too; same implementation on both ends"
    echo "   is the most reliable way to get cross-machine discovery.)"
fi
unset CYCLONEDDS_URI
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="$DOMAIN_ID"
export ROS_LOCALHOST_ONLY=0
echo "  RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "  ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "  -> Jetson'da da ayni ROS_DOMAIN_ID ($DOMAIN_ID) ve rmw_fastrtps_cpp olmali."

echo
echo "== 2) Baglanti testi (ping $JETSON_IP) =="
if ping -c 2 -W 2 "$JETSON_IP" > /dev/null 2>&1; then
    echo "  OK - $JETSON_IP erisilebilir."
else
    echo "  HATA: $JETSON_IP'ye ping atilamadi. Kablo / arayuz / IP'yi kontrol et:"
    ip -brief addr show
    return 1 2>/dev/null || exit 1
fi

echo
echo "== 3) ros2 daemon sifirlaniyor (eski discovery cache'ini temizler) =="
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start

echo
echo "== 4) Multicast testi =="
echo "  Jetson'da SIMDI ayri bir terminalde su komutu calistir:"
echo "    ros2 multicast receive"
echo "  Buradan 5 saniye multicast gonderiliyor..."
timeout 5 ros2 multicast send || true
echo "  Jetson'da 'Received from ...' gormediysen, Jetson'da ufw ac:"
echo "    sudo ufw allow from $(ip route get "$JETSON_IP" 2>/dev/null | awk '{for(i=1;i<=NF;i++) if ($i==\"src\") print $(i+1)}')/24"

echo
echo "== Hazir =="
echo "  ros2 topic list  ->  Jetson'daki topicleri de gormelisin."
