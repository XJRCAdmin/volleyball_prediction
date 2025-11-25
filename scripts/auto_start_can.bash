#!/bin/bash
# 自启动can
DIR="(cd"(dirname "${BASH_SOURCE[0]}")" && pwd)"
gnome-terminal --title "mapping.sh" – bash -c \ 
"echo '618618' | sudo -S bash -c 'cd "$DIR" && ./activate_can.sh'; exec bash"
